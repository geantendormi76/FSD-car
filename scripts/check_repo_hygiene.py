#!/usr/bin/env python3
import hashlib
import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REQUIRED_FILES = (
    "README.md",
    "HANDOFF.md",
    "docs/evidence/final_metrics.json",
    "model/warehouse_nav14_candidate.json",
    "model/warehouse_nav14_candidate.onnx",
    "model/xfeat_640x640.onnx",
    "contracts/phase6/phase6_status.json",
    "contracts/phase7/phase7_status.json",
    "contracts/real_vehicle/pre_hardware_status.json",
)
FORBIDDEN_COMPONENTS = {"target", "out", "dataset", "artifacts"}
OBSOLETE_README_CLAIMS = (
    "Spiced Self-Play RL",
    "零样本、零真实数据、一字不改",
    "百元级边缘算力",
)


def forbidden_tracked_path(path):
    normalized = path.replace("\\", "/").lstrip("./")
    if normalized.startswith("docs/evidence/"):
        return False
    return bool(FORBIDDEN_COMPONENTS.intersection(normalized.split("/")))


def git_lines(*args):
    result = subprocess.run(
        ["git", *args], cwd=ROOT, check=True, capture_output=True, text=True
    )
    return [line for line in result.stdout.splitlines() if line]


def model_hash_errors(root, manifest):
    errors = []
    for model in manifest.get("active_models", []):
        path = Path(root) / model["path"]
        if not path.is_file():
            errors.append(f"active model is missing: {model['path']}")
            continue
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != model["sha256"]:
            errors.append(f"active model hash mismatch: {model['path']}")
    return errors


def main():
    errors = []
    for path in REQUIRED_FILES:
        if not (ROOT / path).is_file():
            errors.append(f"required delivery file is missing: {path}")

    for path in git_lines("ls-files"):
        if forbidden_tracked_path(path):
            errors.append(f"generated/private path is tracked: {path}")

    for path in REQUIRED_FILES:
        if path.endswith(".onnx"):
            ignored = subprocess.run(
                ["git", "check-ignore", "-q", path], cwd=ROOT, check=False
            ).returncode == 0
            if ignored:
                errors.append(f"active deployment model is ignored: {path}")

    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for claim in OBSOLETE_README_CLAIMS:
        if claim in readme:
            errors.append(f"README contains obsolete validated-as-false claim: {claim}")

    delivery = json.loads((ROOT / "model/DELIVERY.json").read_text(encoding="utf-8"))
    errors.extend(model_hash_errors(ROOT, delivery))

    if errors:
        raise SystemExit("\n".join(f"ERROR: {error}" for error in errors))
    print("Repository hygiene validation OK")
    print(f"{len(REQUIRED_FILES)} delivery files present; active models are publishable")


if __name__ == "__main__":
    main()
