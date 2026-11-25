"""Launch Isaac Sim with a sampled, conservative VRAM guard."""
import argparse
import os
import signal
import subprocess
import sys
from pathlib import Path


def vram_used():
    output = subprocess.check_output(
        ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
        text=True, timeout=3, stderr=subprocess.PIPE)
    return max(int(value) for value in output.splitlines())


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", type=Path, default=root/"output/wheel_cell.usdc")
    parser.add_argument("--budget-gib", type=int, choices=range(1, 9), default=8,
                        metavar="1..8")
    cameras = parser.add_mutually_exclusive_group()
    cameras.add_argument("--assembly", dest="camera", action="store_const", const="Assembly",
                         help="Use the assembly cell camera")
    cameras.add_argument("--wheel-detail", dest="camera", action="store_const", const="WheelDetail",
                         help="Use the front-left wheel camera")
    cameras.add_argument("--hub-detail", dest="camera", action="store_const", const="HubDetail",
                         help="Use the bare-hub close-up camera")
    cameras.add_argument("--top", dest="camera", action="store_const", const="TopView",
                         help="Use the orthographic top view")
    parser.set_defaults(camera="Overview")
    parser.add_argument("--replay", action="store_true", help="Use the original timeline preview")
    parser.add_argument("--seed", type=int, help="Reproduce the wheel arrival angles")
    parser.add_argument("--jam", action="store_true", help="Inject a 2 mm FL guide offset in live mode")
    parser.add_argument("--start-at", type=float, default=None, metavar="SECONDS",
                        help="Seek to this scene time when starting")
    parser.add_argument("--quality", choices=("balanced", "high", "cinematic"), default="high")
    parser.add_argument("--smoke-frames", type=int, default=0, help="Render N updates headless, then exit")
    args = parser.parse_args()
    replay = args.replay or args.start_at is not None
    if replay and (args.seed is not None or args.jam):
        parser.error("--seed and --jam require live PhysX mode (omit --replay and --start-at)")
    if args.smoke_frames < 0:
        parser.error("--smoke-frames must be nonnegative")
    sim_dir = Path(os.environ.get("ISAAC_SIM_ROOT", Path.home()/"isaacsim"))
    binary = sim_dir/"python.sh"
    if not binary.is_file() or not args.stage.is_file():
        parser.error("Set ISAAC_SIM_ROOT or build the USD with build.sh before launching.")
    limit = args.budget_gib * 1024
    peak = 0
    process = None
    try:
        peak = vram_used()
        if peak >= limit:
            print(f"GPU already uses {peak} MiB (budget: {limit} MiB).", file=sys.stderr)
            return 2
        flags = ["--camera", args.camera, "--start-at", str(args.start_at or 0.), "--quality", args.quality]
        if replay:
            flags.append("--replay")
        if args.seed is not None:
            flags.extend(("--seed",str(args.seed)))
        if args.jam:
            flags.append("--jam")
        process = subprocess.Popen([str(binary), "-B", str(root/"play.py"),
                                    str(args.stage.resolve()), str(args.smoke_frames), *flags],
                                   start_new_session=True)
        while True:
            try:
                return process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                peak = max(peak, vram_used())
                if peak >= limit:
                    print(f"VRAM budget reached: {peak} MiB / {limit} MiB.", file=sys.stderr)
                    return 2
    except (subprocess.SubprocessError, OSError, ValueError) as error:
        detail = getattr(error, "stderr", None) or getattr(error, "stdout", None) or str(error)
        print(f"Cannot monitor NVIDIA VRAM: {detail.strip()}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        if process is not None:
            stop(process)
            print(f"Peak sampled total VRAM on any card: {peak} MiB (budget {limit} MiB).")


if __name__ == "__main__":
    sys.exit(main())
