# Asset and Code Provenance

This document records the origin and licensing of third-party components
incorporated into the Mooney M20M Bravo aircraft.

The aircraft as distributed by this repository is licensed under the
GNU General Public License version 2 or later. See `COPYING`.

## Primary aircraft contributors

### Philips Nguyen

Contributions include:

- JSBSim flight dynamics
- aerodynamic model
- mass and balance model
- propulsion integration
- Lycoming TIO-540-AF1B simulation work
- propeller and governor simulation work
- aircraft systems
- FlightGear integration
- testing and validation

### Emmanuel Baranger (Helijah)

Contributions include:

- Mooney exterior and interior model assets
- textures
- cockpit and visual assets
- subsequent model-related work

### Israel Emmanuel (Naviat)

Contributions include:

- model integration
- integration of the Helijah model package into this aircraft

## Incorporated FlightGear components

### Legacy Canvas MAP

Files:

- `Nasal/map.nas`
- `Nasal/littleaircraftRed.svg`

Origin:

The MAP implementation derives from legacy FlightGear aircraft code.
A substantially matching implementation exists in the FlightGear
C-160 Transall aircraft and derives from an earlier Mirage-2000 MAP
implementation.

The original source contains the attribution:

    thanks to Harbal1

The identity and exact contribution of Harbal1 have not been independently
established, so the original attribution is preserved verbatim.

License:

GNU General Public License version 2 through the upstream FlightGear
aircraft packages.

The implementation was restored and adapted for the Mooney M20M Bravo
by Philips Nguyen in 2026.

### Bendix-King KMA24

Files:

- `Models/Interior/Panel/Instruments/kma24/`

Contributors identified in the incorporated files:

- Torsten Dreyer — original instrument definition, December 2008
- Emmanuel Baranger — later updates
- Richard Senior — Nasal implementation
- Jackie Reyes — Nasal implementation

The `KMA24.nas` implementation carries copyright notices for
Richard Senior and Jackie Reyes and is licensed under the
GNU General Public License version 2 or later.

### Bendix-King KT76A

Files:

- `Models/Interior/Panel/Instruments/kt76a/`

Copyright:

- Richard Senior

License:

GNU General Public License version 2 or later.

### ARC EA-401A altimeter

Files:

- `Models/Interior/Panel/Instruments/Altimeter/`

Original author:

- Pavel Cueto

Later updates:

- Emmanuel Baranger

License:

GPL.

### Bendix/King KX155 COMM/NAV

Files:

- `Models/Interior/Panel/Instruments/kx155/`

Original and adapting contributors named by the upstream files:

- Torsten Dreyer
- Pavel Cueto
- Fernando Espinosa

Later updates:

- Emmanuel Baranger

License:

GPL.

## Technical reference material

Copyrighted maintenance manuals, service manuals, drawings, and other
reference documentation may be consulted during development for factual
aircraft dimensions, specifications, operating data, and engineering
information.

Copyrighted source documents, pages, figures, and scans are not distributed
as part of this project.

In particular, Figure 6-1 from the Mooney M20M Service & Maintenance Manual
was used as private development reference material and is intentionally
excluded from this repository.
