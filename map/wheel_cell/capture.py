"""Render documentation screenshots of the replay scene from its named cameras.

Usage: ~/isaacsim/python.sh capture.py <stage.usda> <out_dir> [camera@seconds ...]
"""
import sys
from pathlib import Path

from isaacsim import SimulationApp

stage_path, out_dir = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
shots = sys.argv[3:] or ["Overview@40", "TopView@40", "Assembly@45", "WheelDetail@44", "HubDetail@20"]
out_dir.mkdir(parents=True, exist_ok=True)

app = SimulationApp({"headless": True, "width": 1600, "height": 900, "renderer": "RayTracedLighting",
                     "anti_aliasing": 1, "multi_gpu": False, "max_gpu_count": 1})
import omni.timeline
import omni.usd
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Sdf

context = omni.usd.get_context()
context.open_stage(str(stage_path))
while context.get_stage_loading_status()[2]:
    app.update()
viewport = get_active_viewport()
viewport.set_texture_resolution((1600, 900))
timeline = omni.timeline.get_timeline_interface()
for _ in range(60):
    app.update()
for shot in shots:
    camera, seconds = shot.split("@")
    viewport.camera_path = Sdf.Path(f"/World/{camera}")
    timeline.set_current_time(float(seconds))
    for _ in range(90):  # let RTX accumulate and textures stream in
        app.update()
    file = out_dir/f"{camera.lower()}_{float(seconds):g}s.png"
    file.unlink(missing_ok=True)
    capture = capture_viewport_to_file(viewport, str(file))
    for _ in range(1500):  # the capture is written asynchronously
        app.update()
        if file.exists():
            break
    print("CAPTURED" if file.exists() else "MISSING", file, flush=True)
app.close()
