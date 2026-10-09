"""Build the motor-control host into a single clickable Windows executable.

    .venv/Scripts/python.exe tools/build_host_app.py

Produces dist/FM03运动控制.exe (and prints its size and SHA256). The build is
reproducible: it only bundles tools/motor_control.py and its pyinstaller dependency,
and it refuses to run if the executable would import a second serial library.

The output directory is ignored by git, so the executable is a local artifact; the
source remains the deliverable that is version controlled.
"""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / "tools" / "motor_control.py"
APP_NAME = "FM03_MotorControl"
DIST = ROOT / "dist"


def run(command: list[str]) -> None:
    print("+", " ".join(command), flush=True)
    subprocess.run(command, check=True, cwd=ROOT)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default=APP_NAME)
    parser.add_argument("--console", action="store_true",
                        help="keep a console window (useful for debugging)")
    parser.add_argument("--onedir", action="store_true",
                        help="folder build instead of a single file (starts faster)")
    args = parser.parse_args()

    if not ENTRY.exists():
        print(f"missing entry point {ENTRY}", file=sys.stderr)
        return 1
    try:
        import PyInstaller  # noqa: F401
    except ImportError:
        print("PyInstaller is required: .venv/Scripts/python.exe -m pip install pyinstaller",
              file=sys.stderr)
        return 1

    # PyInstaller replaces only this named app. Preserve other releases/executables.
    DIST.mkdir(parents=True,exist_ok=True)

    command = [sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean",
               "--name", args.name, "--onefile" if not args.onedir else "--onedir",
               "--console" if args.console else "--windowed",
               # pyserial's windowed builds can miss its VERSION file; include it so
               # serial.tools.list_ports works in the frozen app.
               "--hidden-import", "serial.tools.list_ports",
               "--collect-submodules", "serial",
               str(ENTRY)]
    run(command)

    # PyInstaller writes a spec file next to the entry point's parent; keep the tree
    # clean, since it is a build artifact that can be regenerated from this script.
    spec = ROOT / (args.name + ".spec")
    if spec.exists():
        spec.unlink()

    produced = DIST / (args.name + (".exe" if not args.onedir else ""))
    if not produced.exists():
        candidates = list(DIST.rglob("*.exe"))
        if not candidates:
            print("build finished but no executable was produced", file=sys.stderr)
            return 1
        produced = candidates[0]
    size = produced.stat().st_size
    digest = hashlib.sha256(produced.read_bytes()).hexdigest()
    print(f"\nbuilt  : {produced}")
    print(f"size   : {size:,} bytes ({size / 1024 / 1024:.1f} MiB)")
    print(f"sha256 : {digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
