#!/usr/bin/env python3
"""Closed-loop parameter proposal interface for simulation tuning.

The tuner does not assume how Gazebo or the goal GUI is launched. It writes a
parameter file and a JSON trial request; an external runner executes that trial
and writes the scorer JSON back. SPSA then uses two trials to estimate a local
gradient and proposes the next parameter vector.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import random
from pathlib import Path
from typing import Any


def _yaml():
    try:
        import yaml
        return yaml
    except ImportError as exc:
        raise SystemExit("sim_tuner requires PyYAML; install python3-yaml") from exc


def _get(tree: dict[str, Any], path: str) -> Any:
    value: Any = tree
    for part in path.split("."):
        value = value[part]
    return value


def _set(tree: dict[str, Any], path: str, value: Any) -> None:
    parts = path.split(".")
    target = tree
    for part in parts[:-1]:
        target = target[part]
    target[parts[-1]] = value


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def load_space(path: Path, params: Path) -> tuple[dict[str, Any], dict[str, dict[str, float]]]:
    yaml = _yaml()
    config = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    raw_specs = config.get("parameters", config)
    base = yaml.safe_load(params.read_text(encoding="utf-8")) or {}
    specs: dict[str, dict[str, float]] = {}
    for name, raw in raw_specs.items():
        if not isinstance(raw, dict) or "min" not in raw or "max" not in raw:
            raise ValueError(f"参数 {name} 必须包含 min 和 max")
        low, high = float(raw["min"]), float(raw["max"])
        initial = float(raw.get("initial", _get(base, name)))
        if not math.isfinite(low + high + initial) or low >= high or not low <= initial <= high:
            raise ValueError(f"参数 {name} 的范围或 initial 无效")
        specs[name] = {"min": low, "max": high, "initial": initial}
    return base, specs


def _actual(theta: dict[str, float], specs: dict[str, dict[str, float]]) -> dict[str, float]:
    return {
        name: spec["min"] + theta[name] * (spec["max"] - spec["min"])
        for name, spec in specs.items()
    }


def _write_params(base: dict[str, Any], values: dict[str, float], path: Path) -> None:
    yaml = _yaml()
    output = copy.deepcopy(base)
    for name, value in values.items():
        _set(output, name, value)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(output, sort_keys=False, allow_unicode=True), encoding="utf-8")


def _loss(result_path: Path) -> float:
    result = json.loads(result_path.read_text(encoding="utf-8"))
    score = result.get("score")
    if result.get("success") is False:
        return 1.0 + min(1.0, float(result.get("collision_count", 0)))
    if score is None:
        return 2.0
    return _clip(1.0 - float(score) / 100.0, 0.0, 2.0)


def init(args: argparse.Namespace) -> None:
    base, specs = load_space(args.space, args.base_params)
    theta = {
        name: (spec["initial"] - spec["min"]) / (spec["max"] - spec["min"])
        for name, spec in specs.items()
    }
    state = {
        "version": 1,
        "environment": "simulation",
        "base_params": str(args.base_params.resolve()),
        "space": str(args.space.resolve()),
        "output_dir": str(args.output_dir.resolve()),
        "routes": args.routes,
        "repeats": args.repeats,
        "specs": specs,
        "theta": theta,
        "iteration": 0,
        "seed": args.seed,
        "a": args.a,
        "c": args.c,
        "A": args.A,
        "alpha": args.alpha,
        "gamma": args.gamma,
        "pending": None,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.state.parent.mkdir(parents=True, exist_ok=True)
    args.state.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"state": str(args.state), "parameters": _actual(theta, specs)}, ensure_ascii=False, indent=2))


def propose(args: argparse.Namespace) -> None:
    state = json.loads(args.state.read_text(encoding="utf-8"))
    if state.get("pending"):
        raise SystemExit("上一次 propose 还没有 tell，不能生成下一组实验")
    specs = state["specs"]
    theta = {name: float(value) for name, value in state["theta"].items()}
    iteration = int(state["iteration"])
    rng = random.Random(int(state["seed"]) + iteration)
    delta = {name: (1.0 if rng.random() >= 0.5 else -1.0) for name in specs}
    ck = float(state["c"]) / ((iteration + 1) ** float(state["gamma"]))
    plus = {name: _clip(theta[name] + ck * delta[name], 0.0, 1.0) for name in specs}
    minus = {name: _clip(theta[name] - ck * delta[name], 0.0, 1.0) for name in specs}
    trial_dir = Path(state["output_dir"]) / f"iteration_{iteration:04d}"
    base = _yaml().safe_load(Path(state["base_params"]).read_text(encoding="utf-8")) or {}
    plus_params, minus_params = trial_dir / "plus_params.yaml", trial_dir / "minus_params.yaml"
    _write_params(base, _actual(plus, specs), plus_params)
    _write_params(base, _actual(minus, specs), minus_params)
    pending = {
        "iteration": iteration,
        "c": ck,
        "delta": delta,
        "routes": state.get("routes", []),
        "repeats": state.get("repeats", 1),
        "seed": int(state["seed"]) + iteration,
        "plus": {"trial_id": f"{iteration:04d}_plus", "params_file": str(plus_params), "result_file": str(trial_dir / "plus_result.json")},
        "minus": {"trial_id": f"{iteration:04d}_minus", "params_file": str(minus_params), "result_file": str(trial_dir / "minus_result.json")},
    }
    state["pending"] = pending
    args.state.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(pending, ensure_ascii=False, indent=2))


def tell(args: argparse.Namespace) -> None:
    state = json.loads(args.state.read_text(encoding="utf-8"))
    pending = state.get("pending")
    if not pending:
        raise SystemExit("当前没有等待 tell 的实验")
    plus_loss, minus_loss = _loss(args.plus_result), _loss(args.minus_result)
    theta = {name: float(value) for name, value in state["theta"].items()}
    specs = state["specs"]
    gradient = {
        name: (plus_loss - minus_loss) / (2.0 * pending["c"] * pending["delta"][name])
        for name in specs
    }
    k = int(state["iteration"])
    ak = float(state["a"]) / ((float(state["A"]) + k + 1.0) ** float(state["alpha"]))
    updated = {name: _clip(theta[name] - ak * gradient[name], 0.0, 1.0) for name in specs}
    state["theta"] = updated
    state["iteration"] = k + 1
    state["pending"] = None
    state["last_update"] = {"plus_loss": plus_loss, "minus_loss": minus_loss, "gradient": gradient, "step": ak}
    args.state.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"iteration": k + 1, "parameters": _actual(updated, specs), "loss": state["last_update"]}, ensure_ascii=False, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description="仿真 Nav2 闭环参数调节接口")
    sub = parser.add_subparsers(dest="command", required=True)
    p_init = sub.add_parser("init", help="创建调参状态"); p_init.add_argument("--base-params", type=Path, required=True); p_init.add_argument("--space", type=Path, required=True); p_init.add_argument("--state", type=Path, required=True); p_init.add_argument("--output-dir", type=Path, required=True); p_init.add_argument("--routes", nargs="*", default=[], help="每次 trial 使用的固定路线名称"); p_init.add_argument("--repeats", type=int, default=3, help="每条路线重复次数"); p_init.add_argument("--seed", type=int, default=2026); p_init.add_argument("--a", type=float, default=0.15); p_init.add_argument("--c", type=float, default=0.08); p_init.add_argument("--A", type=float, default=10.0); p_init.add_argument("--alpha", type=float, default=0.602); p_init.add_argument("--gamma", type=float, default=0.101)
    p_propose = sub.add_parser("propose", help="生成一对 SPSA 实验"); p_propose.add_argument("--state", type=Path, required=True)
    p_tell = sub.add_parser("tell", help="读取一对评分并更新参数"); p_tell.add_argument("--state", type=Path, required=True); p_tell.add_argument("--plus-result", type=Path, required=True); p_tell.add_argument("--minus-result", type=Path, required=True)
    args = parser.parse_args()
    {"init": init, "propose": propose, "tell": tell}[args.command](args)


if __name__ == "__main__":
    main()
