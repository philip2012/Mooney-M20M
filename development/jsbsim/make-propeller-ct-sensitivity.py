#!/usr/bin/env python3

from pathlib import Path
import argparse
import shutil
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]

SOURCE_ENGINES = (
    REPO / "Engines"
)

PROP_NAME = "M20M-Propeller.xml"

PITCH_COLUMNS = (
    15.1,
    25.0,
    30.0,
    35.0,
    40.0,
    44.5,
)

GAIN_START_PITCH = 30.0
GAIN_END_PITCH = 44.5


def parse_table(table):
    data = table.find("tableData")

    if data is None or not data.text:
        raise RuntimeError(
            "tableData missing"
        )

    rows = [
        line.split()
        for line in data.text.splitlines()
        if line.strip()
    ]

    header = [
        float(v)
        for v in rows[0]
    ]

    if len(header) != 2:
        raise RuntimeError(
            "Expected original two-pitch table"
        )

    output = []

    for row in rows[1:]:
        output.append(
            (
                float(row[0]),
                float(row[1]),
                float(row[2]),
            )
        )

    return (
        header[0],
        header[1],
        output,
    )


def interpolate(
    low_value,
    high_value,
    pitch,
    low_pitch,
    high_pitch,
):
    fraction = (
        (pitch - low_pitch)
        /
        (high_pitch - low_pitch)
    )

    return (
        low_value
        + fraction
        * (
            high_value
            - low_value
        )
    )


def ct_gain(
    pitch,
    end_gain,
):
    if pitch <= GAIN_START_PITCH:
        return 1.0

    if pitch >= GAIN_END_PITCH:
        return end_gain

    fraction = (
        (pitch - GAIN_START_PITCH)
        /
        (
            GAIN_END_PITCH
            - GAIN_START_PITCH
        )
    )

    return (
        1.0
        + fraction
        * (
            end_gain
            - 1.0
        )
    )


def rebuild_table(
    table,
    end_gain,
    modify_ct,
):
    (
        low_pitch,
        high_pitch,
        rows,
    ) = parse_table(table)

    lines = []

    lines.append(
        "            "
        + " ".join(
            f"{pitch:.1f}"
            for pitch in PITCH_COLUMNS
        )
    )

    for (
        advance,
        low_value,
        high_value,
    ) in rows:

        values = []

        for pitch in PITCH_COLUMNS:
            value = interpolate(
                low_value,
                high_value,
                pitch,
                low_pitch,
                high_pitch,
            )

            if modify_ct:
                value *= ct_gain(
                    pitch,
                    end_gain,
                )

            values.append(
                value
            )

        lines.append(
            f"            {advance:.1f} "
            + " ".join(
                f"{value:.6f}"
                for value in values
            )
        )

    table.find(
        "tableData"
    ).text = (
        "\n"
        + "\n".join(lines)
        + "\n        "
    )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "gain",
        type=float,
    )

    parser.add_argument(
        "output",
        type=Path,
    )

    args = parser.parse_args()

    if args.gain < 1.0:
        raise SystemExit(
            "gain must be >= 1.0"
        )

    output = (
        args.output.resolve()
    )

    if output.exists():
        shutil.rmtree(
            output
        )

    shutil.copytree(
        SOURCE_ENGINES,
        output,
    )

    prop_path = (
        output / PROP_NAME
    )

    tree = ET.parse(
        prop_path
    )

    root = tree.getroot()

    ct_table = None
    cp_table = None

    for table in root.findall(
        "table"
    ):
        name = table.attrib.get(
            "name"
        )

        if name == "C_THRUST":
            ct_table = table

        elif name == "C_POWER":
            cp_table = table

    if ct_table is None:
        raise SystemExit(
            "C_THRUST not found"
        )

    if cp_table is None:
        raise SystemExit(
            "C_POWER not found"
        )

    # Reconstruct both surfaces using extra pitch
    # columns. With gain=1.0 this should be exactly
    # equivalent to the original linear two-column
    # interpolation.
    rebuild_table(
        ct_table,
        args.gain,
        True,
    )

    rebuild_table(
        cp_table,
        args.gain,
        False,
    )

    tree.write(
        prop_path,
        encoding="UTF-8",
        xml_declaration=True,
    )

    print(
        f"Created sensitivity candidate: "
        f"{output}"
    )

    print(
        f"High-pitch CT end gain: "
        f"{args.gain:.3f}"
    )

    print(
        "C_POWER unchanged."
    )

    print(
        "C_THRUST unchanged through 30 deg."
    )


if __name__ == "__main__":
    main()
