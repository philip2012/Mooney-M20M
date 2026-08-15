#!/usr/bin/env python3

"""
Authoritative reference data for the Lycoming TIO-540-AF1A/AF1B.

This file contains source data only. It is NOT controller logic and must
not be imported into the permanent aircraft FDM.

Primary source:
    Lycoming TIO-540 Series Parallel Valve Cylinder Heads
    Operator's Manual
    P/N 60297-23P
    4th Edition, March 2006
    Revision 60297-23P-2, October 2006

Relevant figures:
    Figure 3-31 - Sea Level/Altitude Performance, 2575 RPM
    Figure 3-32 - Sea Level/Altitude Performance, 2400 RPM
    Figure 3-33 - Sea Level/Altitude Performance, 2200 RPM
    Figure 3-34 - Maximum Manifold Pressure vs Altitude

Important:
    Figures 3-31 through 3-33 specify ZERO RAM ALTITUDE PERFORMANCE.

The post-critical Figure 3-34 points below are visually digitized from
the straight maximum-cumulative-MAP segment. The graph has 1 inHg and
1,000 ft grid spacing. Treat graph-derived MAP values as approximately
+/- 0.1 inHg rather than fake-precision test-cell values.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class MapReferencePoint:
    altitude_ft: float
    map_inhg: float
    uncertainty_inhg: float
    source: str


@dataclass(frozen=True)
class PowerReferencePoint:
    altitude_ft: float
    hp_nominal: float | None
    hp_min: float | None
    hp_max: float | None
    status: str
    source: str


# FAA/Lycoming certification anchor.
#
# AF1A/AF1B:
#   270 HP
#   2575 RPM
#   standard-density critical altitude 20,000 ft
#   36.5 inHg
CERTIFIED_CRITICAL_ALTITUDE_FT = 20000.0
CERTIFIED_CRITICAL_MAP_INHG = 36.5
CERTIFIED_RATED_RPM = 2575.0
CERTIFIED_RATED_HP = 270.0


# Figure 3-34:
# "MAXIMUM CUMULATIVE MANIFOLD PRESSURE"
#
# The descending post-critical segment is visually a straight line from
# about 38.0 inHg at 19,000 ft to 29.0 inHg at 25,000 ft.
#
# The 20,000-ft point is independently anchored by certification at
# 36.5 inHg.
FIGURE_3_34_POST_CRITICAL_2575 = (
    MapReferencePoint(
        19000.0,
        38.0,
        0.1,
        "Lycoming 60297-23P Figure 3-34",
    ),
    MapReferencePoint(
        20000.0,
        36.5,
        0.1,
        "Lycoming Figure 3-34 + certified critical-altitude anchor",
    ),
    MapReferencePoint(
        21000.0,
        35.0,
        0.1,
        "Lycoming 60297-23P Figure 3-34",
    ),
    MapReferencePoint(
        22000.0,
        33.5,
        0.1,
        "Lycoming 60297-23P Figure 3-34",
    ),
    MapReferencePoint(
        23000.0,
        32.0,
        0.1,
        "Lycoming 60297-23P Figure 3-34",
    ),
    MapReferencePoint(
        24000.0,
        30.5,
        0.1,
        "Lycoming 60297-23P Figure 3-34",
    ),
    MapReferencePoint(
        25000.0,
        29.0,
        0.1,
        "Lycoming 60297-23P Figure 3-34",
    ),
)


# AF1A/AF1B 2575-RPM high-altitude power references.
#
# Figure 3-31 is a graphical zero-ram performance chart rather than
# a numerical data table. Do not assign fake precision to chart reads.
#
# 20,000 ft:
#   exact certification/rated point.
#
# 22,000 ft:
#   approximately 250 HP from Figure 3-31.
#
# 24,000 ft:
#   approximately 225-230 HP from Figure 3-31.
#
# Figure 3-31's altitude plot does not provide a defensible 25,000-ft
# HP point, so none is included here.
AF1B_2575_POWER_REFERENCE = (
    PowerReferencePoint(
        20000.0,
        270.0,
        None,
        None,
        "exact",
        "AF1A/AF1B certification rated point",
    ),
    PowerReferencePoint(
        22000.0,
        250.0,
        None,
        None,
        "approximate-chart-read",
        "Lycoming 60297-23P Figure 3-31 / Curve 13491",
    ),
    PowerReferencePoint(
        24000.0,
        None,
        225.0,
        230.0,
        "graphical-range",
        "Lycoming 60297-23P Figure 3-31 / Curve 13491",
    ),
)


def figure_3_34_post_critical_map(
    altitude_ft: float,
) -> float:
    """
    Linear representation of the published straight post-critical
    Figure 3-34 maximum-cumulative-MAP segment.

    Valid only from 19,000 through 25,000 ft.

        19,000 ft -> 38.0 inHg
        slope     -> -1.5 inHg / 1,000 ft

    This is diagnostic/reference interpolation, not permanent FDM logic.
    """

    if not 19000.0 <= altitude_ft <= 25000.0:
        raise ValueError(
            "Figure 3-34 post-critical reference is valid only "
            "from 19,000 through 25,000 ft"
        )

    return (
        38.0
        - 1.5
        * (
            altitude_ft
            - 19000.0
        )
        / 1000.0
    )


if __name__ == "__main__":
    print(
        "Lycoming TIO-540-AF1A/AF1B "
        "Figure 3-34 post-critical reference"
    )
    print()
    print(
        " Altitude    MAP"
    )
    print(
        "----------------"
    )

    for point in FIGURE_3_34_POST_CRITICAL_2575:
        calculated = figure_3_34_post_critical_map(
            point.altitude_ft
        )

        print(
            f"{point.altitude_ft:7.0f} ft  "
            f"{point.map_inhg:5.2f}\"  "
            f"calc={calculated:5.2f}\""
        )
