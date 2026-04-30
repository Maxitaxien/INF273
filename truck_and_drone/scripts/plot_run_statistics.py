#!/usr/bin/env python3

from __future__ import annotations

import csv
import html
import math
import sys
from collections import defaultdict
from pathlib import Path

PALETTE = [
    "#111111",
    "#1f77b4",
    "#d62728",
    "#2ca02c",
    "#ff7f0e",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#7f7f7f",
    "#bcbd22",
]


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def escape(text: object) -> str:
    return html.escape(str(text), quote=True)


def sanitize_filename(name: str) -> str:
    cleaned = []
    for ch in name.strip():
        if ch.isalnum():
            cleaned.append(ch.lower())
        elif ch in {" ", "-", "_"}:
            cleaned.append("_")
    result = "".join(cleaned).strip("_")
    return result or "plot"


def parse_float(value: str) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def format_number(value: float) -> str:
    abs_value = abs(value)
    if abs_value >= 1000 or value == math.floor(value):
        return f"{value:.0f}"
    if abs_value >= 10:
        return f"{value:.1f}"
    return f"{value:.2f}"


def dataset_key(name: str) -> tuple[str, int]:
    name = name.upper()
    dataset_type = name[0]
    digits = "".join(ch for ch in name if ch.isdigit())
    return dataset_type, int(digits)


def is_primary_dataset_name(name: str) -> bool:
    if not name:
        return False
    if name[0].upper() not in {"F", "R"}:
        return False
    digits = name[2:] if len(name) > 1 and name[1] == "_" else name[1:]
    return digits.isdigit()


def load_dict_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def parse_summary_result(path: Path, algorithm: str) -> dict[str, object] | None:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.reader(handle))

    if len(rows) < 3:
        return None

    headers = rows[1]
    values = rows[2]
    if headers and "Average" not in headers[0]:
        headers = headers[1:]
        values = values[1:]
    mapping = {header.strip(): value.strip() for header, value in zip(headers, values)}
    dataset = path.stem
    if not is_primary_dataset_name(dataset):
        return None

    dataset_type, size = dataset_key(dataset)
    return {
        "dataset": dataset,
        "dataset_type": dataset_type,
        "size": size,
        "algorithm": algorithm,
        "average_objective": parse_float(mapping.get("Average objective", "")),
        "best_objective": parse_float(mapping.get("Best Objective", "")),
        "improvement_percent": parse_float(mapping.get("Improvement (%)", "")),
        "average_runtime": parse_float(mapping.get("Average running time (in seconds)", "")),
    }


def svg_header(width: int, height: int, title: str) -> list[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        f"<title>{escape(title)}</title>",
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="white" />',
    ]


def write_svg(path: Path, lines: list[str]) -> None:
    path.write_text("\n".join(lines + ["</svg>"]) + "\n", encoding="utf-8")


def numeric_ticks(min_value: float, max_value: float, max_ticks: int = 6) -> list[float]:
    if math.isclose(min_value, max_value):
        return [min_value]
    count = max(2, max_ticks)
    step = (max_value - min_value) / (count - 1)
    return [min_value + i * step for i in range(count)]


def bounds_from_series(series: list[dict[str, object]]) -> tuple[float, float, float, float] | None:
    xs: list[float] = []
    ys: list[float] = []
    for item in series:
        for x, y in item["points"]:
            xs.append(float(x))
            ys.append(float(y))
    if not xs or not ys:
        return None
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    if math.isclose(min_x, max_x):
        min_x -= 1.0
        max_x += 1.0
    if math.isclose(min_y, max_y):
        pad = max(1.0, abs(min_y) * 0.1)
        min_y -= pad
        max_y += pad
    else:
        pad = (max_y - min_y) * 0.08
        min_y -= pad
        max_y += pad
    return min_x, max_x, min_y, max_y


def downsample_xy_points(points: list[tuple[float, float]], max_points: int) -> list[tuple[float, float]]:
    if len(points) <= max_points or max_points <= 2:
        return points

    selected_indices = {0, len(points) - 1}
    remaining = max_points - len(selected_indices)
    if remaining <= 0:
        return [points[0], points[-1]]

    step = (len(points) - 1) / float(remaining + 1)
    for idx in range(1, remaining + 1):
        selected_indices.add(int(round(idx * step)))

    return [points[idx] for idx in sorted(selected_indices)]


def downsample_objective_points(
    points: list[tuple[float, float, float]],
    max_points: int = 900,
) -> list[tuple[float, float, float]]:
    if len(points) <= max_points:
        return points

    selected_indices = {0, len(points) - 1}
    prev_best = None
    for idx, (_, _, best) in enumerate(points):
        if prev_best is None or best < prev_best:
            selected_indices.add(idx)
            prev_best = best

    remaining_budget = max_points - len(selected_indices)
    if remaining_budget > 0:
        available = [idx for idx in range(len(points)) if idx not in selected_indices]
        if available:
            step = len(available) / float(remaining_budget)
            for sample_idx in range(remaining_budget):
                picked = available[min(len(available) - 1, int(sample_idx * step))]
                selected_indices.add(picked)

    selected = sorted(selected_indices)
    if len(selected) > max_points:
        trimmed = downsample_xy_points(
            [(float(idx), float(idx)) for idx in selected],
            max_points)
        kept_indices = {int(x) for x, _ in trimmed}
        selected = [idx for idx in selected if idx in kept_indices]

    return [points[idx] for idx in selected]


def write_line_or_scatter_chart(
    path: Path,
    title: str,
    x_label: str,
    y_label: str,
    series: list[dict[str, object]],
    *,
    scatter_only: bool = False,
    show_markers: bool = True,
    width: int = 1100,
    height: int = 620,
) -> None:
    bounds = bounds_from_series(series)
    if bounds is None:
        return

    min_x, max_x, min_y, max_y = bounds
    left = 90
    right = 260
    top = 70
    bottom = 80
    plot_width = width - left - right
    plot_height = height - top - bottom

    def map_x(value: float) -> float:
        return left + (value - min_x) * plot_width / (max_x - min_x)

    def map_y(value: float) -> float:
        return top + plot_height - (value - min_y) * plot_height / (max_y - min_y)

    lines = svg_header(width, height, title)
    lines.append(
        f'<text x="{width / 2:.1f}" y="34" text-anchor="middle" font-size="24" '
        f'font-family="Arial, sans-serif">{escape(title)}</text>'
    )

    for tick in numeric_ticks(min_y, max_y):
        y = map_y(tick)
        lines.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}" '
            f'stroke="#dddddd" stroke-width="1" />'
        )
        lines.append(
            f'<text x="{left - 10}" y="{y + 4:.2f}" text-anchor="end" font-size="12" '
            f'font-family="Arial, sans-serif">{escape(format_number(tick))}</text>'
        )

    unique_x = sorted({point[0] for item in series for point in item["points"]})
    x_ticks = unique_x if len(unique_x) <= 10 else numeric_ticks(min_x, max_x)
    for tick in x_ticks:
        x = map_x(float(tick))
        lines.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_height}" '
            f'stroke="#f0f0f0" stroke-width="1" />'
        )
        tick_label = str(int(tick)) if abs(tick - round(tick)) < 1e-9 else format_number(float(tick))
        lines.append(
            f'<text x="{x:.2f}" y="{top + plot_height + 24}" text-anchor="middle" '
            f'font-size="12" font-family="Arial, sans-serif">{escape(tick_label)}</text>'
        )

    lines.append(
        f'<rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" '
        f'fill="none" stroke="#333333" stroke-width="1.5" />'
    )
    lines.append(
        f'<text x="{left + plot_width / 2:.1f}" y="{height - 20}" text-anchor="middle" '
        f'font-size="15" font-family="Arial, sans-serif">{escape(x_label)}</text>'
    )
    lines.append(
        f'<text x="24" y="{top + plot_height / 2:.1f}" text-anchor="middle" '
        f'font-size="15" font-family="Arial, sans-serif" '
        f'transform="rotate(-90 24 {top + plot_height / 2:.1f})">{escape(y_label)}</text>'
    )

    legend_x = left + plot_width + 24
    legend_y = top + 18

    for index, item in enumerate(series):
        color = item["color"]
        points = item["points"]
        label = item["label"]
        if not points:
            continue

        if not scatter_only and len(points) >= 2:
            coordinates = " ".join(f"{map_x(float(x)):.2f},{map_y(float(y)):.2f}" for x, y in points)
            lines.append(
                f'<polyline points="{coordinates}" fill="none" stroke="{color}" '
                f'stroke-width="2.4" />'
            )

        if show_markers:
            for x_value, y_value in points:
                cx = map_x(float(x_value))
                cy = map_y(float(y_value))
                lines.append(
                    f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="3.2" fill="{color}" '
                    f'fill-opacity="0.75" />'
                )

        legend_entry_y = legend_y + index * 24
        lines.append(
            f'<line x1="{legend_x}" y1="{legend_entry_y}" x2="{legend_x + 18}" y2="{legend_entry_y}" '
            f'stroke="{color}" stroke-width="2.4" />'
        )
        lines.append(
            f'<circle cx="{legend_x + 9}" cy="{legend_entry_y}" r="3.2" fill="{color}" />'
        )
        lines.append(
            f'<text x="{legend_x + 28}" y="{legend_entry_y + 4}" font-size="13" '
            f'font-family="Arial, sans-serif">{escape(label)}</text>'
        )

    write_svg(path, lines)


def write_horizontal_bar_chart(
    path: Path,
    title: str,
    x_label: str,
    entries: list[tuple[str, float, str]],
) -> None:
    if not entries:
        return

    width = 1100
    height = 110 + max(1, len(entries)) * 38
    left = 260
    right = 70
    top = 70
    bottom = 60
    plot_width = width - left - right
    max_value = max(value for _, value, _ in entries)
    if max_value <= 0:
        max_value = 1.0

    def map_x(value: float) -> float:
        return left + value * plot_width / max_value

    lines = svg_header(width, height, title)
    lines.append(
        f'<text x="{width / 2:.1f}" y="34" text-anchor="middle" font-size="24" '
        f'font-family="Arial, sans-serif">{escape(title)}</text>'
    )

    for tick in numeric_ticks(0.0, max_value):
        x = map_x(tick)
        lines.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{height - bottom}" '
            f'stroke="#efefef" stroke-width="1" />'
        )
        lines.append(
            f'<text x="{x:.2f}" y="{height - bottom + 22}" text-anchor="middle" '
            f'font-size="12" font-family="Arial, sans-serif">{escape(format_number(tick))}</text>'
        )

    lines.append(
        f'<rect x="{left}" y="{top}" width="{plot_width}" height="{height - top - bottom}" '
        f'fill="none" stroke="#333333" stroke-width="1.5" />'
    )
    lines.append(
        f'<text x="{left + plot_width / 2:.1f}" y="{height - 16}" text-anchor="middle" '
        f'font-size="15" font-family="Arial, sans-serif">{escape(x_label)}</text>'
    )

    for index, (label, value, color) in enumerate(entries):
        y = top + 10 + index * 38
        bar_height = 22
        bar_width = map_x(value) - left
        lines.append(
            f'<text x="{left - 10}" y="{y + bar_height - 4}" text-anchor="end" '
            f'font-size="13" font-family="Arial, sans-serif">{escape(label)}</text>'
        )
        lines.append(
            f'<rect x="{left}" y="{y}" width="{bar_width:.2f}" height="{bar_height}" '
            f'fill="{color}" fill-opacity="0.85" />'
        )
        lines.append(
            f'<text x="{left + bar_width + 8:.2f}" y="{y + bar_height - 4}" '
            f'font-size="12" font-family="Arial, sans-serif">{escape(format_number(value))}</text>'
        )

    write_svg(path, lines)


def aggregate_operator_runtime(trace_rows: list[dict[str, str]]) -> list[dict[str, object]]:
    aggregated: dict[str, dict[str, object]] = {}
    for row in trace_rows:
        name = row.get("operator_name", "")
        if not name:
            continue
        entry = aggregated.setdefault(
            name,
            {
                "operator_name": name,
                "uses": 0,
                "total_runtime_ms": 0.0,
                "accepted_moves": 0,
            },
        )
        entry["uses"] += 1
        entry["total_runtime_ms"] += parse_float(row.get("runtime_ms", "")) or 0.0
    results = []
    for entry in aggregated.values():
        uses = max(1, int(entry["uses"]))
        results.append(
            {
                "operator_name": entry["operator_name"],
                "uses": entry["uses"],
                "total_runtime_ms": entry["total_runtime_ms"],
                "avg_runtime_ms": entry["total_runtime_ms"] / uses,
            }
        )
    return results


def write_operator_summary_csv(path: Path, operator_rows: list[dict[str, object]]) -> None:
    if not operator_rows:
        return
    fieldnames = list(operator_rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in operator_rows:
            writer.writerow(row)


def normalize_operator_rows(operator_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    normalized: list[dict[str, object]] = []
    for row in operator_rows:
        name = row.get("operator_name", "")
        if not name:
            continue
        normalized.append(
            {
                "operator_name": str(name),
                "uses": int(parse_float(str(row.get("uses", 0))) or 0),
                "successful_calls": int(parse_float(str(row.get("successful_calls", 0))) or 0),
                "failures": int(parse_float(str(row.get("failures", 0))) or 0),
                "infeasible_candidates": int(parse_float(str(row.get("infeasible_candidates", 0))) or 0),
                "feasible_candidates": int(parse_float(str(row.get("feasible_candidates", 0))) or 0),
                "accepted_moves": int(parse_float(str(row.get("accepted_moves", 0))) or 0),
                "improving_accepts": int(parse_float(str(row.get("improving_accepts", 0))) or 0),
                "new_bests": int(parse_float(str(row.get("new_bests", 0))) or 0),
                "delta_samples": int(parse_float(str(row.get("delta_samples", 0))) or 0),
                "delta_sum": parse_float(str(row.get("delta_sum", 0))) or 0.0,
                "avg_delta": parse_float(str(row.get("avg_delta", 0))) or 0.0,
                "total_runtime_ms": parse_float(str(row.get("total_runtime_ms", 0))) or 0.0,
                "avg_runtime_ms": parse_float(str(row.get("avg_runtime_ms", 0))) or 0.0,
            }
        )
    normalized.sort(key=lambda row: row["avg_runtime_ms"], reverse=True)
    return normalized


def write_dataset_operator_summary(path: Path, dataset_name: str, operator_rows: list[dict[str, object]]) -> None:
    if not operator_rows:
        return

    lines = [
        f"# Slowest operators for {dataset_name}",
        "",
        "Sorted by average runtime per use from the saved GAM best-run statistics.",
        "",
        "| Rank | Operator | Avg runtime (ms) | Total runtime (ms) | Uses | Accepted | Improving accepts | New bests |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]

    for rank, row in enumerate(operator_rows, start=1):
        lines.append(
            f"| {rank} | {row['operator_name']} | "
            f"{row['avg_runtime_ms']:.3f} | {row['total_runtime_ms']:.3f} | "
            f"{row['uses']} | {row['accepted_moves']} | "
            f"{row['improving_accepts']} | {row['new_bests']} |"
        )

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_run_operator_summary(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return

    aggregated: dict[str, dict[str, object]] = {}
    for row in rows:
        operator_name = str(row["operator_name"])
        entry = aggregated.setdefault(
            operator_name,
            {
                "operator_name": operator_name,
                "uses": 0,
                "accepted_moves": 0,
                "improving_accepts": 0,
                "new_bests": 0,
                "total_runtime_ms": 0.0,
                "datasets": set(),
            },
        )
        entry["uses"] += int(row["uses"])
        entry["accepted_moves"] += int(row["accepted_moves"])
        entry["improving_accepts"] += int(row["improving_accepts"])
        entry["new_bests"] += int(row["new_bests"])
        entry["total_runtime_ms"] += float(row["total_runtime_ms"])
        entry["datasets"].add(str(row["dataset"]))

    ranked = []
    for entry in aggregated.values():
        uses = max(1, int(entry["uses"]))
        ranked.append(
            {
                "operator_name": entry["operator_name"],
                "datasets": len(entry["datasets"]),
                "uses": entry["uses"],
                "accepted_moves": entry["accepted_moves"],
                "improving_accepts": entry["improving_accepts"],
                "new_bests": entry["new_bests"],
                "total_runtime_ms": entry["total_runtime_ms"],
                "avg_runtime_ms": entry["total_runtime_ms"] / uses,
            }
        )
    ranked.sort(key=lambda row: row["avg_runtime_ms"], reverse=True)

    lines = [
        "# Slowest operators across this run",
        "",
        "Aggregated over all dataset best-run GAM statistics that include operator timing.",
        "",
        "| Rank | Operator | Avg runtime (ms) | Total runtime (ms) | Uses | Datasets | Accepted | Improving accepts | New bests |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]

    for rank, row in enumerate(ranked, start=1):
        lines.append(
            f"| {rank} | {row['operator_name']} | {row['avg_runtime_ms']:.3f} | "
            f"{row['total_runtime_ms']:.3f} | {row['uses']} | {row['datasets']} | "
            f"{row['accepted_moves']} | {row['improving_accepts']} | {row['new_bests']} |"
        )

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def generate_dataset_plots(statistics_dir: Path) -> None:
    trace_rows = load_dict_rows(statistics_dir / "trace.csv")
    weight_rows = load_dict_rows(statistics_dir / "weights.csv")
    run_rows = load_dict_rows(statistics_dir / "runs.csv")
    operator_rows = load_dict_rows(statistics_dir / "operators.csv")

    color_map: dict[str, str] = {}

    def color_for(name: str) -> str:
        if name not in color_map:
            color_map[name] = PALETTE[len(color_map) % len(PALETTE)]
        return color_map[name]

    if trace_rows:
        temperature_series = [{
            "label": "Temperature",
            "color": PALETTE[0],
            "points": [
                (int(row["iteration"]), float(row["temperature"]))
                for row in trace_rows
                if row.get("iteration") and row.get("temperature")
            ],
        }]
        write_line_or_scatter_chart(
            statistics_dir / "temperature.svg",
            f"{statistics_dir.stem} Temperature",
            "Iteration",
            "Temperature",
            temperature_series,
        )

        worsening_series = [{
            "label": "Worsening acceptance probability",
            "color": PALETTE[1],
            "points": [
                (int(row["iteration"]), float(row["worsening_acceptance_probability"]))
                for row in trace_rows
                if parse_float(row.get("worsening_acceptance_probability", "")) is not None
                and float(row["worsening_acceptance_probability"]) >= 0.0
            ],
        }]
        write_line_or_scatter_chart(
            statistics_dir / "worsening_acceptance.svg",
            f"{statistics_dir.stem} Worsening Acceptance",
            "Iteration",
            "Probability",
            worsening_series,
            scatter_only=True,
        )

        operator_delta_series: list[dict[str, object]] = []
        grouped_deltas: dict[str, list[tuple[int, float]]] = defaultdict(list)
        for row in trace_rows:
            if row.get("has_delta") != "1":
                continue
            delta = parse_float(row.get("delta", ""))
            iteration = parse_float(row.get("iteration", ""))
            operator_name = row.get("operator_name", "")
            if delta is None or iteration is None or not operator_name:
                continue
            grouped_deltas[operator_name].append((int(iteration), delta))

        for operator_name, points in grouped_deltas.items():
            operator_delta_series.append(
                {
                    "label": operator_name,
                    "color": color_for(operator_name),
                    "points": points,
                }
            )

        write_line_or_scatter_chart(
            statistics_dir / "operator_deltas.svg",
            f"{statistics_dir.stem} Objective Delta by Operator",
            "Iteration",
            "Delta",
            operator_delta_series,
            scatter_only=True,
        )

        delta_dir = statistics_dir / "delta_by_operator"
        ensure_dir(delta_dir)
        for operator_name, points in grouped_deltas.items():
            write_line_or_scatter_chart(
                delta_dir / f"{sanitize_filename(operator_name)}.svg",
                f"{statistics_dir.stem} {operator_name} Deltas",
                "Iteration",
                "Delta",
                [{
                    "label": operator_name,
                    "color": color_for(operator_name),
                    "points": points,
                }],
                scatter_only=True,
            )

    if weight_rows:
        grouped_weights: dict[str, list[tuple[int, float]]] = defaultdict(list)
        for row in weight_rows:
            segment = parse_float(row.get("segment", ""))
            weight = parse_float(row.get("weight", ""))
            operator_name = row.get("operator_name", "")
            if segment is None or weight is None or not operator_name:
                continue
            grouped_weights[operator_name].append((int(segment), weight))

        weight_series = [
            {
                "label": operator_name,
                "color": color_for(operator_name),
                "points": points,
            }
            for operator_name, points in grouped_weights.items()
        ]
        write_line_or_scatter_chart(
            statistics_dir / "weights.svg",
            f"{statistics_dir.stem} Operator Weights",
            "Segment",
            "Weight",
            weight_series,
        )

    if run_rows:
        run_series = [{
            "label": "Final objective",
            "color": PALETTE[2],
            "points": [
                (int(row["run"]), float(row["final_objective"]))
                for row in run_rows
                if row.get("run") and row.get("final_objective")
            ],
        }]
        write_line_or_scatter_chart(
            statistics_dir / "run_objectives.svg",
            f"{statistics_dir.stem} Run Objectives",
            "Run",
            "Final objective",
            run_series,
        )

    trace_has_runtime = bool(trace_rows) and "runtime_ms" in trace_rows[0]
    if not operator_rows and trace_rows and trace_has_runtime:
        operator_rows = aggregate_operator_runtime(trace_rows)
        write_operator_summary_csv(statistics_dir / "operators_derived.csv", operator_rows)

    if operator_rows:
        normalized_operator_rows = normalize_operator_rows(operator_rows)
        write_dataset_operator_summary(
            statistics_dir / "slowest_operators.md",
            statistics_dir.stem,
            normalized_operator_rows,
        )

        runtime_entries: list[tuple[str, float, str]] = []
        for row in normalized_operator_rows:
            runtime_entries.append(
                (str(row["operator_name"]), float(row["avg_runtime_ms"]), color_for(str(row["operator_name"])))
            )
        write_horizontal_bar_chart(
            statistics_dir / "operator_runtime.svg",
            f"{statistics_dir.stem} Operator Runtime",
            "Average runtime per use (ms)",
            runtime_entries,
        )


def generate_run_summary_plots(run_dir: Path) -> None:
    summary_rows: list[dict[str, object]] = []
    for algorithm_dir in sorted(run_dir.iterdir()):
        if not algorithm_dir.is_dir():
            continue
        algorithm_name = algorithm_dir.name
        for csv_path in sorted(algorithm_dir.glob("*.csv")):
            parsed = parse_summary_result(csv_path, algorithm_name)
            if parsed is not None:
                summary_rows.append(parsed)

    if not summary_rows:
        return

    plots_dir = run_dir / "plots"
    ensure_dir(plots_dir)

    metrics = [
        ("best_objective", "Best Objective", "Objective"),
        ("average_objective", "Average Objective", "Objective"),
        ("improvement_percent", "Improvement", "Percent"),
        ("average_runtime", "Average Runtime", "Seconds"),
    ]

    for dataset_type in ("F", "R"):
        family_rows = [row for row in summary_rows if row["dataset_type"] == dataset_type]
        if not family_rows:
            continue

        for metric_key, metric_label, y_label in metrics:
            grouped: dict[str, list[tuple[int, float]]] = defaultdict(list)
            for row in family_rows:
                value = row.get(metric_key)
                if value is None:
                    continue
                grouped[str(row["algorithm"])].append((int(row["size"]), float(value)))

            series = []
            for index, (algorithm_name, points) in enumerate(sorted(grouped.items())):
                points.sort(key=lambda item: item[0])
                series.append(
                    {
                        "label": algorithm_name,
                        "color": PALETTE[index % len(PALETTE)],
                        "points": points,
                    }
                )

            write_line_or_scatter_chart(
                plots_dir / f"{dataset_type.lower()}_{sanitize_filename(metric_key)}.svg",
                f"{dataset_type}-Instance {metric_label}",
                "Instance size",
                y_label,
                series,
            )

    run_operator_rows: list[dict[str, object]] = []
    for algorithm_dir in sorted(run_dir.iterdir()):
        if not algorithm_dir.is_dir():
            continue
        for statistics_dir in sorted(algorithm_dir.glob("*_statistics")):
            operator_rows = load_dict_rows(statistics_dir / "operators.csv")
            trace_rows = load_dict_rows(statistics_dir / "trace.csv")
            if not operator_rows and trace_rows and "runtime_ms" in trace_rows[0]:
                operator_rows = aggregate_operator_runtime(trace_rows)
            for row in normalize_operator_rows(operator_rows):
                row["dataset"] = statistics_dir.stem
                row["algorithm"] = algorithm_dir.name
                run_operator_rows.append(row)

    write_run_operator_summary(plots_dir / "slowest_operators.md", run_operator_rows)


def generate_all_plots(run_dir: Path) -> None:
    generate_run_summary_plots(run_dir)

    for algorithm_dir in sorted(run_dir.iterdir()):
        if not algorithm_dir.is_dir():
            continue
        for statistics_dir in sorted(algorithm_dir.glob("*_statistics")):
            if statistics_dir.is_dir():
                generate_dataset_plots(statistics_dir)


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("Usage: plot_run_statistics.py <run_dir>", file=sys.stderr)
        return 1

    run_dir = Path(argv[1]).resolve()
    if not run_dir.exists():
        print(f"Run directory not found: {run_dir}", file=sys.stderr)
        return 1

    generate_all_plots(run_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
