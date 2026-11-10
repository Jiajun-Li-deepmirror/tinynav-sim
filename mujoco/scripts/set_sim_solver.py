#!/usr/bin/env python3
"""Write the contact solver a policy was trained with into its ONNX metadata.

    python set_sim_solver.py --cone elliptic --impratio 10 <policy.onnx>...

Sets `sim_cone` and `sim_impratio` (sim/contract_v2.py; the runtime puts them
on the model when it binds the policy), leaving the graph and every other key
as they are. For exports made before the exporters wrote them: read the
values from the training run's params/env.yaml (sim.mujoco cone / impratio).
Needs onnx (not a sim dependency).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import onnx


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("onnx", type=Path, nargs="+")
    parser.add_argument("--cone", choices=("pyramidal", "elliptic"), required=True)
    parser.add_argument("--impratio", type=float, required=True)
    args = parser.parse_args()
    for path in args.onnx:
        model = onnx.load(path)
        meta = {p.key: p.value for p in model.metadata_props}
        meta.update({"sim_cone": args.cone, "sim_impratio": f"{args.impratio:.6g}"})
        del model.metadata_props[:]
        for key, value in meta.items():
            model.metadata_props.add(key=key, value=value)
        onnx.save(model, path)
        print(f"{path}: sim_cone {args.cone}, sim_impratio {args.impratio:g}")


if __name__ == "__main__":
    main()
