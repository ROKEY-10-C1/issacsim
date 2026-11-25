"""Run inside Isaac Sim's Python environment."""
import argparse
import sqlite3
from pathlib import Path

QUALITY = {
    "balanced": (1600, 900, "RaytracedLighting"),
    "high": (1920, 1080, "RaytracedLighting"),
    "cinematic": (2560, 1440, "PathTracing"),
}


def live(app, stage, args):
    import secrets
    import omni.timeline
    import omni.ui as ui
    import omni.usd
    from pxr import Sdf, Usd

    from physics import Cell

    seed = args.seed if args.seed is not None else secrets.randbits(32)
    print(f"Live wheel cell seed={seed}", flush=True)
    context = omni.usd.get_context()
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_start_time(0.)
    timeline.set_end_time(stage.GetEndTimeCode()/stage.GetTimeCodesPerSecond())
    timeline.set_looping(False)
    session = stage.GetSessionLayer()
    live_layer = Sdf.Layer.CreateAnonymous("wheel_cell_live.usda")
    session.subLayerPaths.insert(0, live_layer.identifier)
    try:
        with Usd.EditContext(stage, live_layer):
            cell = Cell(stage, seed, args.jam, manual=False)
            pre = cell.api.subscribe_physics_on_step_events(lambda dt: cell.prepare_step(dt), True, 0)
            post = cell.api.subscribe_physics_on_step_events(lambda dt: cell.finish_step(dt), False, 0)
            try:
                window = ui.Window("CN7 | Live PhysX", width=555, height=194) if not args.smoke_frames else None
                labels = {}

                def toggle():
                    if timeline.is_playing():
                        timeline.pause()
                    else:
                        timeline.play()

                if window:
                    with window.frame, ui.VStack(spacing=4):
                        button = ui.Button("Play", clicked_fn=toggle)
                        for name in ("clock", "FL", "FR", "RL", "RR"):
                            labels[name] = ui.Label("")
                if args.smoke_frames:
                    timeline.play()
                frames = 0
                while app.is_running() and context.get_stage() == stage and (not args.smoke_frames or frames<args.smoke_frames):
                    for t, name, phase in cell.pending:
                        print(f"{t:6.2f}s {name} {phase}", flush=True)
                    cell.pending.clear()
                    if labels:
                        button.text = "Pause" if timeline.is_playing() else "Play"
                        update_live_status(cell, labels, seed)
                    app.update()
                    frames += 1
                    if context.get_stage() == stage and timeline.is_stopped() and cell.seconds:
                        live_layer.Clear()
                        cell = Cell(stage, seed, args.jam, manual=False)
            finally:
                pre.unsubscribe()
                post.unsubscribe()
                if context.get_stage() == stage:
                    timeline.stop()
                    app.update()
    finally:
        session.subLayerPaths.remove(live_layer.identifier)


def update_live_status(cell, labels, seed):
    amr = "positioned" if cell.carrier.ready else "dock / lift" if cell.carrier.docked else "travel"
    labels["clock"].text = f"CPU PhysX   {cell.seconds:.1f} s   |   AMR {amr}   |   seed {seed}"
    for unit in cell.stations:
        if unit.phase in ("clamp", "tip_up", "clamp_failed", "grip_slip", "tip_failed"):
            details = (f"pad force {unit.pad_force_step:.0f} N  |  "
                       f"max slip {unit.max_grip_slip*1000:.1f} mm")
        else:
            details = f"phase {unit.error:+.2f}°  |  gap {unit.gap*1000:+.1f} mm"
        labels[unit.name].text = f"{unit.name}  {unit.phase}  |  {details}"


def play(app, args):
    import omni.timeline
    import omni.usd
    from omni.kit.viewport.utility import get_active_viewport
    from pxr import Sdf

    context = omni.usd.get_context()
    context.open_stage(str(args.stage))
    while context.get_stage_loading_status()[2]:
        app.update()
    stage = context.get_stage()
    if stage is None or not stage.GetPrimAtPath("/World/Assembly"):
        raise RuntimeError(f"Unable to open wheel cell stage {args.stage}")

    viewport = get_active_viewport()
    if viewport:
        viewport.camera_path = Sdf.Path(f"/World/{args.camera}")
        viewport.set_texture_resolution(QUALITY[args.quality][:2])
    physics_setup=bool(stage.GetPrimAtPath("/World/Live/Physics"))
    if physics_setup and (args.jam or args.seed is not None):
        raise ValueError("Use wheel_cell.usdc for live control with --seed or --jam.")
    if physics_setup and args.start_at:
        raise ValueError("Start the physical stage at time zero so the AMR can follow the inbound route.")
    if not args.replay and not physics_setup:
        live(app,stage,args)
        return
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_start_time(0)
    end = stage.GetEndTimeCode()/stage.GetTimeCodesPerSecond()
    timeline.set_end_time(end)
    if not 0 <= args.start_at < end:
        raise ValueError(f"Start time must be within [0, {end:.1f}) seconds")
    timeline.set_looping(not physics_setup)

    events, labels = [], {}
    window = None
    if not args.smoke_frames and not physics_setup:
        import omni.ui as ui

        db = args.stage.with_name("production.sqlite3")
        if db.is_file():
            with sqlite3.connect(db.as_uri()+"?mode=ro", uri=True) as connection:
                events = connection.execute(
                    "SELECT sim_s, vehicle, station, code FROM events ORDER BY sim_s").fetchall()
            window = ui.Window("Wheel Cell", width=345, height=157)
            with window.frame:
                with ui.VStack(spacing=4):
                    labels = {key: ui.Label("") for key in ("time", "FL", "FR", "RL", "RR")}

    timeline.set_current_time(args.start_at)
    if not physics_setup or args.smoke_frames:
        timeline.play()
    tick = -1
    count = 0
    while app.is_running() and (not args.smoke_frames or count < args.smoke_frames):
        app.update()
        count += 1
        now = timeline.get_current_time()
        if window is None or int(now*5) == tick:
            continue
        tick = int(now*5)
        vehicle = None
        recent = {}
        for seconds, order, station, code in events:
            if seconds > now:
                break
            if order != vehicle:
                recent.clear()
                vehicle = order
            if station:
                recent[station] = code
        labels["time"].text = f"{now:.1f} s  |  {vehicle or 'Waiting for first wheel'}"
        for key in ("FL", "FR", "RL", "RR"):
            labels[key].text = f"{key}   {recent.get(key, 'waiting')}"


def main():
    from isaacsim import SimulationApp

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", type=Path)
    parser.add_argument("smoke_frames", type=int)
    parser.add_argument("--camera", choices=("Overview", "Assembly", "WheelDetail", "HubDetail", "TopView"), default="Overview")
    parser.add_argument("--start-at", type=float, default=0.)
    parser.add_argument("--quality", choices=QUALITY, default="high")
    parser.add_argument("--replay", action="store_true")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--jam", action="store_true")
    args = parser.parse_args()
    width, height, renderer = QUALITY[args.quality]
    path_tracing = renderer == "PathTracing"
    app = SimulationApp({"headless": bool(args.smoke_frames), "multi_gpu": False,
                         "max_gpu_count": 1, "width": width, "height": height,
                         "window_width": 1600, "window_height": 960,
                         "renderer": renderer, "denoiser": path_tracing,
                         "samples_per_pixel_per_frame": 4 if path_tracing else 1,
                         "max_bounces": 6, "anti_aliasing": 1,
                         "extra_args": ["--/rtx/raytracingMotion/enabled=false"]})
    try:
        play(app, args)
    finally:
        app.close()


if __name__ == "__main__":
    main()
