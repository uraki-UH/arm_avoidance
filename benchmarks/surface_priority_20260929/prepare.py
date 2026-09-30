#!/usr/bin/env python3
"""固定入力復元と微動円柱の合成、比較条件の生成。"""
import copy
import gzip
import hashlib
import json
import math
from pathlib import Path
import shutil
import cbor2

workspace = Path(__file__).resolve().parents[2]
bench = workspace / "benchmarks/surface_priority_20260929"
output = workspace / "artifacts/surface_priority_20260929"
inputs = output / "inputs"
inputs.mkdir(parents=True, exist_ok=True)
saved = workspace / "artifacts/surface_local_20260929/perf/inputs"
provenance = {}
for name in ("input", "small", "large_curve"):
    source = saved / (name + ".cbor.gz")
    target = inputs / (name + ".cbor")
    with gzip.open(source, "rb") as stream, target.open("wb") as restored:
        shutil.copyfileobj(stream, restored)
    provenance[name] = {"source": str(source.relative_to(workspace)),
                        "sha256": hashlib.sha256(source.read_bytes()).hexdigest()}

# 元円柱の剛体微動。遠方平面は固定、半径と接続と所属は不変。
base = cbor2.loads((inputs / "large_curve.cbor").read_bytes())[0]
records = []
for frame_idx in range(40):
    record = copy.deepcopy(base)
    record["map"]["frame_number"] = frame_idx + 1
    angle = 0.003 * math.sin(frame_idx * 0.21)
    shift = 0.001 * math.sin(frame_idx * 0.17)
    cos_value, sin_value = math.cos(angle), math.sin(angle)
    for node in record["map"]["nodes"][:288]:
        x, z = node[2], node[4]
        nx, nz = node[5], node[7]
        node[2] = cos_value * x + sin_value * z + shift
        node[4] = -sin_value * x + cos_value * z
        node[5] = cos_value * nx + sin_value * nz
        node[7] = -sin_value * nx + cos_value * nz
    records.append(record)
(inputs / "micro_curve.cbor").write_bytes(cbor2.dumps(records))
provenance["micro_curve"] = {"base": "large_curve", "frames": 40,
    "curve_nodes": 288, "radius_m": 0.1, "rotation_y_rad": 0.003,
    "translation_x_m": 0.001}
(output / "input_provenance.json").write_text(json.dumps(provenance, indent=2))
cases = []
for name in ("small", "large_curve", "input", "micro_curve"):
    for mode in ("before", "on", "off"):
        cases.append({"name": name + "_" + mode,
            "argv": ["python3", str(bench / "case.py"), "--input", name,
                "--mode", mode, "--output", "@case_dir@"],
            "metrics": "metrics.json", "env": {"PYTHONDONTWRITEBYTECODE": "1"}})
(output / "cases.json").write_text(json.dumps({"cases": cases}, indent=2))
print(json.dumps({"num_cases": len(cases), "output": str(output)}))
