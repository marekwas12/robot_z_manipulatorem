#!/usr/bin/env python3
import os
import shutil
from datetime import datetime

import numpy as np
import yaml

SAMPLES_FILE = os.path.expanduser("~/robot_ws/cube_grasp_samples.yaml")
OUTPUT_FILE = os.path.expanduser("~/robot_ws/cube_grasp_calibration.yaml")

TARGET_RMS_M = 0.008
TARGET_MAX_M = 0.015
MIN_KEEP = 8


def fit(samples):
    camera = np.asarray([s["camera"] for s in samples], dtype=np.float64)
    tool = np.asarray([s["tool"] for s in samples], dtype=np.float64)

    design = np.hstack(
        [camera, np.ones((len(samples), 1), dtype=np.float64)]
    )

    coeff, _, rank, _ = np.linalg.lstsq(design, tool, rcond=None)
    if rank < 4:
        raise RuntimeError(
            "Probki sa geometrycznie zbyt podobne; rank < 4."
        )

    predicted = design @ coeff
    errors_xyz = predicted - tool
    errors = np.linalg.norm(errors_xyz, axis=1)

    rms = float(np.sqrt(np.mean(errors ** 2)))
    max_error = float(np.max(errors))
    axis_rms = np.sqrt(np.mean(errors_xyz ** 2, axis=0))

    matrix = coeff[:3, :].T
    offset = coeff[3, :]

    return {
        "matrix": matrix,
        "offset": offset,
        "errors": errors,
        "rms": rms,
        "max": max_error,
        "axis_rms": axis_rms,
    }


def main():
    if not os.path.exists(SAMPLES_FILE):
        raise SystemExit(f"Brak pliku: {SAMPLES_FILE}")

    with open(SAMPLES_FILE, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    samples = data.get("samples", [])
    if len(samples) < MIN_KEEP:
        raise SystemExit(
            f"Za malo probek: {len(samples)}. Minimum {MIN_KEEP}."
        )

    # Zachowujemy oryginalny numer probki do logu.
    working = []
    for i, sample in enumerate(samples, start=1):
        item = dict(sample)
        item["_original_index"] = i
        working.append(item)

    # Backup zanim cokolwiek usuniemy.
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = SAMPLES_FILE + f".backup_{stamp}"
    shutil.copy2(SAMPLES_FILE, backup)
    print(f"Backup: {backup}")

    removed = []

    # Najpierw wykrywamy prawie identyczny CAMERA z mocno innym TOOL.
    # Z takiej pary wyrzucamy punkt, który gorzej pasuje do modelu
    # zbudowanego na pozostalych probkach.
    changed = True
    while changed and len(working) > MIN_KEEP:
        changed = False

        for i in range(len(working)):
            ci = np.asarray(working[i]["camera"], dtype=float)
            ti = np.asarray(working[i]["tool"], dtype=float)

            for j in range(i + 1, len(working)):
                cj = np.asarray(working[j]["camera"], dtype=float)
                tj = np.asarray(working[j]["tool"], dtype=float)

                camera_dist = float(np.linalg.norm(ci - cj))
                tool_dist = float(np.linalg.norm(ti - tj))

                if camera_dist < 0.002 and tool_dist > 0.012:
                    # Porownaj oba punkty wzgledem modelu bez tej pary.
                    rest = [
                        s for k, s in enumerate(working)
                        if k not in (i, j)
                    ]

                    if len(rest) >= MIN_KEEP - 1:
                        model = fit(rest)

                        def pred_err(sample):
                            c = np.asarray(sample["camera"], dtype=float)
                            t = np.asarray(sample["tool"], dtype=float)
                            p = model["matrix"] @ c + model["offset"]
                            return float(np.linalg.norm(p - t))

                        ei = pred_err(working[i])
                        ej = pred_err(working[j])
                        bad = i if ei >= ej else j
                    else:
                        bad = j

                    removed.append(working[bad])
                    print(
                        "Usuwam sprzeczna duplikacje CAMERA: "
                        f"oryginalna probka "
                        f"{working[bad]['_original_index']}"
                    )
                    working.pop(bad)
                    changed = True
                    break
            if changed:
                break

    # Potem iteracyjnie wyrzucamy najgorszy residual.
    while len(working) > MIN_KEEP:
        result = fit(working)

        print(
            f"Przed pruningiem: n={len(working)} "
            f"RMS={result['rms']*1000:.1f} mm "
            f"MAX={result['max']*1000:.1f} mm"
        )

        if (
            result["rms"] <= TARGET_RMS_M
            and
            result["max"] <= TARGET_MAX_M
        ):
            break

        worst = int(np.argmax(result["errors"]))
        bad = working.pop(worst)
        removed.append(bad)

        print(
            f"  -> usuwam najgorsza oryginalna probke "
            f"{bad['_original_index']} "
            f"(residual={result['errors'][worst]*1000:.1f} mm)"
        )

    result = fit(working)

    print("")
    print("==========================================")
    print("WYNIK PO ODRZUCENIU OUTLIEROW")
    print(f"Zostalo probek: {len(working)}")
    print(
        "Usuniete oryginalne numery:",
        [s["_original_index"] for s in removed],
    )
    print(f"RMS = {result['rms']*1000:.1f} mm")
    print(f"MAX = {result['max']*1000:.1f} mm")
    print(
        "AXIS RMS = "
        f"X:{result['axis_rms'][0]*1000:.1f} "
        f"Y:{result['axis_rms'][1]*1000:.1f} "
        f"Z:{result['axis_rms'][2]*1000:.1f} mm"
    )
    print("==========================================")

    clean_samples = []
    for s in working:
        clean_samples.append(
            {
                "camera": [float(v) for v in s["camera"]],
                "tool": [float(v) for v in s["tool"]],
            }
        )

    clean_data = dict(data)
    clean_data["samples"] = clean_samples

    with open(SAMPLES_FILE, "w", encoding="utf-8") as f:
        yaml.safe_dump(clean_data, f, sort_keys=False)

    calibration = {
        "model": "affine_camera_to_grasp_tcp",
        "formula": "tool_base = matrix * cube_camera + offset",
        "camera_servo_deg": 152.0,
        "sample_count": len(working),
        "matrix": [
            [float(v) for v in row]
            for row in result["matrix"]
        ],
        "offset": [
            float(v) for v in result["offset"]
        ],
        "rms_error_m": result["rms"],
        "max_error_m": result["max"],
        "axis_rms_error_m": {
            "x": float(result["axis_rms"][0]),
            "y": float(result["axis_rms"][1]),
            "z": float(result["axis_rms"][2]),
        },
        "removed_original_sample_numbers": [
            int(s["_original_index"]) for s in removed
        ],
    }

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        yaml.safe_dump(calibration, f, sort_keys=False)

    print(f"Zapisano czyste probki: {SAMPLES_FILE}")
    print(f"Zapisano kalibracje:    {OUTPUT_FILE}")


if __name__ == "__main__":
    main()