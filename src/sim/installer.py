#!/usr/bin/env python3
import os
import sys
import subprocess
from pathlib import Path


def main():
    if len(sys.argv) < 2:
        print("Usage: install.py <package>")
        sys.exit(1)

    pkg = sys.argv[1]

    # Location for automatic virtualenv
    venv_path = Path.home() / ".venvs" / "auto"
    python_bin = venv_path / "bin" / "python"
    pip_bin = venv_path / "bin" / "pip"

    # Create venv if missing
    if not python_bin.exists():
        print(f"[+] Creating virtual environment at {venv_path} ...")
        venv_path.parent.mkdir(parents=True, exist_ok=True)
        subprocess.check_call([sys.executable, "-m", "venv", str(venv_path)])

    # Install package inside the venv
    print(f"[+] Installing '{pkg}' inside {venv_path} ...")
    subprocess.check_call([str(python_bin), "-m", "pip", "install", "--upgrade", "pip"])
    subprocess.check_call([str(pip_bin), "install", pkg])

    print()
    print(f"[✓] Package '{pkg}' installed successfully in virtual environment!")
    print(f"[i] To use it, run:")
    print(f"    source {venv_path}/bin/activate")
    print(f"    python -c 'import {pkg}; print({pkg})'")
    print()


if __name__ == "__main__":
    main()
