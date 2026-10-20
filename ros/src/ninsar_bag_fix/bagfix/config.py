"""Load and merge YAML robot configs.

A config declares an ordered `pipeline` of rules. Each rule names a `transform`
from the registry and carries that transform's options. Optional `bags:`
overrides are deep-merged over the pipeline when the bag name matches.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

DEFAULT_OUTPUT_ROOT = "output"
DEFAULT_OUTPUT_SUFFIX = "_fixed"


class ConfigError(Exception):
    pass


@dataclass
class Rule:
    """One entry of the pipeline: a transform name plus its options."""

    transform: str
    options: dict[str, Any] = field(default_factory=dict)
    rule_id: str = ""

    @property
    def key(self) -> str:
        """Identifier used to address this rule in overrides and --set."""
        return self.rule_id or self.transform

    @property
    def enabled(self) -> bool:
        return bool(self.options.get("enabled", True))


@dataclass
class Config:
    robot: str
    pipeline: list[Rule]
    output_root: str = DEFAULT_OUTPUT_ROOT
    output_suffix: str = DEFAULT_OUTPUT_SUFFIX
    output_storage: str = "mcap"
    #: bytes per split file; None inherits the source's splitting, 0 never splits
    output_split: int | None = None
    convert: dict[str, Any] | None = None
    path: Path | None = None


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge `override` into a copy of `base`."""
    out = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def _coerce_scalar(text: str) -> Any:
    """Parse a --set value using YAML rules (so 'true', '3', '[a, b]' work)."""
    try:
        return yaml.safe_load(text)
    except yaml.YAMLError:
        return text


def _apply_sets(rules: dict[str, dict], sets: list[str]) -> None:
    """Apply `--set rule.option=value` overrides in place."""
    for item in sets:
        if "=" not in item:
            raise ConfigError(f"--set expects rule.option=value, got '{item}'")
        dotted, _, value = item.partition("=")
        rule_key, _, option = dotted.partition(".")
        if not option:
            raise ConfigError(f"--set expects rule.option=value, got '{item}'")
        if rule_key not in rules:
            raise ConfigError(
                f"--set refers to unknown rule '{rule_key}'; "
                f"available: {', '.join(sorted(rules))}"
            )
        rules[rule_key][option] = _coerce_scalar(value)


def load(path: str | Path, bag_name: str = "", sets: list[str] | None = None) -> Config:
    """Read a robot config, applying per-bag overrides and --set arguments."""
    path = Path(path)
    try:
        raw = yaml.safe_load(path.read_text()) or {}
    except FileNotFoundError as exc:
        raise ConfigError(f"config not found: {path}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML in {path}: {exc}") from exc

    if not isinstance(raw, dict):
        raise ConfigError(f"{path}: top level must be a mapping")

    raw_pipeline = raw.get("pipeline") or []
    if not isinstance(raw_pipeline, list):
        raise ConfigError(f"{path}: 'pipeline' must be a list of rules")

    # rules keyed by id/transform name, preserving pipeline order
    rules: dict[str, dict] = {}
    order: list[tuple[str, str]] = []  # (key, transform)
    for index, entry in enumerate(raw_pipeline):
        if not isinstance(entry, dict) or "transform" not in entry:
            raise ConfigError(f"{path}: pipeline[{index}] needs a 'transform' key")
        entry = copy.deepcopy(entry)
        transform = entry.pop("transform")
        key = entry.pop("id", None) or transform
        if key in rules:
            raise ConfigError(
                f"{path}: duplicate rule '{key}'; give one of them a distinct 'id'"
            )
        rules[key] = entry
        order.append((key, transform))

    # YAML turns unquoted date-like keys (2026-01-01) into date objects, and bag
    # directories are usually named after a date - compare on the string form.
    bags = {str(key): value for key, value in (raw.get("bags") or {}).items()}
    overrides = bags.get(bag_name) or {}
    if overrides and not isinstance(overrides, dict):
        raise ConfigError(f"{path}: bags.{bag_name} must be a mapping")
    for key, patch in overrides.items():
        if key not in rules:
            raise ConfigError(
                f"{path}: bags.{bag_name} overrides unknown rule '{key}'; "
                f"available: {', '.join(sorted(rules))}"
            )
        rules[key] = _deep_merge(rules[key], patch or {})

    _apply_sets(rules, sets or [])

    output = raw.get("output") or {}
    return Config(
        robot=raw.get("robot", path.stem),
        pipeline=[
            Rule(transform=transform, options=rules[key], rule_id=key)
            for key, transform in order
        ],
        output_root=output.get("root", DEFAULT_OUTPUT_ROOT),
        output_suffix=output.get("suffix", DEFAULT_OUTPUT_SUFFIX),
        output_storage=output.get("storage", "mcap"),
        output_split=output.get("max_bagfile_size"),
        convert=raw.get("convert"),
        path=path,
    )
