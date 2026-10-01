"""Copy the verified PR source tree into the already-open Raspberry Pi project."""

import argparse
import io
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import time

PR_COMMIT = "e793b9a83a67004dcad940580f7cf6ce6ae0b6af"
REPOSITORY = "https://github.com/Ypa-0739/serialPlotTest.git"
BRANCH = "codex/raspberry-pi-usb-cdc-v2"
PROJECT = Path("/home/f8fq/serialPlotTest")
PREFIX = Path("pi-brain/raspberry-pi")


def merge_configuration(old, new):
    """Retain site-specific values and add newly introduced default fields."""
    if isinstance(old, dict) and isinstance(new, dict):
        return {key: merge_configuration(old[key], value) if key in old else value
                for key, value in new.items()} | {key: value for key, value in old.items() if key not in new}
    return old


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true", help="Back up and update source files")
    args = parser.parse_args()
    project = PROJECT.resolve(strict=True)
    target = project / PREFIX
    if project != PROJECT or target.is_symlink() or not target.resolve().is_relative_to(project):
        raise SystemExit("Unexpected project path or symlink; no files changed")
    subprocess.run(["git", "-C", str(project), "fetch", REPOSITORY, BRANCH], check=True)
    archive = subprocess.check_output(["git", "-C", str(project), "archive", f"{PR_COMMIT}:{PREFIX.as_posix()}"])
    staged_files = {}
    with tarfile.open(fileobj=io.BytesIO(archive)) as source:
        for entry in source.getmembers():
            if entry.isdir():
                continue
            relative = Path(entry.name)
            if not entry.isfile() or relative.is_absolute() or ".." in relative.parts:
                raise SystemExit(f"Unexpected archive entry: {entry.name}")
            destination = target / relative
            if not destination.resolve().is_relative_to(target.resolve()):
                raise SystemExit(f"Destination leaves target: {relative}")
            staged_files[relative] = source.extractfile(entry).read()
    if len(staged_files) != 143:
        raise SystemExit(f"Unexpected source file count: {len(staged_files)}")
    print(f"TARGET={target}\nCOMMIT={PR_COMMIT}\nSOURCE_FILES={len(staged_files)}")
    config_changes = []
    for relative, new_bytes in list(staged_files.items()):
        destination = target / relative
        if relative.parts[0] == "config" and relative.suffix == ".json" and destination.is_file():
            old = json.loads(destination.read_text(encoding="utf-8"))
            new = json.loads(new_bytes.decode("utf-8"))
            if relative.name == "rpi_binary_protocol.json":
                continue  # Contract data is a source artifact, not site configuration.
            merged = merge_configuration(old, new)
            if relative.name == "stm32.json":
                merged["protocol_version"] = 2
            elif relative.name == "navigation.json":
                planner = merged.setdefault("planner", {})
                planner["control_mode"] = "stm32_pose_goal"
                planner["maximum_speed_mm_s"] = min(float(planner["maximum_speed_mm_s"]), 300)
                planner["maximum_yaw_rate_mrad_s"] = min(float(planner["maximum_yaw_rate_mrad_s"]), 800)
            staged_files[relative] = (json.dumps(merged, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
            config_changes.append(relative.as_posix())
    print("PRESERVED_CONFIG=" + json.dumps(config_changes, ensure_ascii=False))
    print("Existing serial port, camera IDs, calibration and model paths are preserved.")
    if not args.apply:
        print("PREVIEW_ONLY: rerun with --apply to sync after creating a backup.")
        return
    backup = project / ("raspberry-pi-source-backup-" + time.strftime("%Y%m%d-%H%M%S"))
    backup.mkdir(exist_ok=False)
    target.mkdir(parents=True, exist_ok=True)
    changed = 0
    for relative, content in staged_files.items():
        destination = target / relative
        if destination.is_file() and destination.read_bytes() == content:
            continue
        if destination.exists():
            backup_file = backup / relative
            backup_file.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(destination, backup_file)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as temporary:
            temporary.write(content)
            temporary_path = Path(temporary.name)
        mode = destination.stat().st_mode & 0o777 if destination.exists() else 0o644
        temporary_path.chmod(mode)
        temporary_path.replace(destination)
        if destination.read_bytes() != content:
            raise SystemExit(f"Verification failed: {relative}; backup: {backup}")
        changed += 1
    receipt = {"commit": PR_COMMIT, "target": str(target), "changed_files": changed,
               "preserved_config": config_changes, "backup": str(backup)}
    (backup / "sync-receipt.json").write_text(json.dumps(receipt, ensure_ascii=False, indent=2), encoding="utf-8")
    print("SYNC_OK=" + json.dumps(receipt, ensure_ascii=False))


if __name__ == "__main__":
    main()
