# Mooney M20M Bravo — FlightGear

Work-in-progress Mooney M20M Bravo for FlightGear.

The Mooney M20M Bravo is a high-performance, turbocharged, retractable-gear four-seat piston aircraft from Mooney’s long-body M20 family. With cruise speeds exceeding 200 KTAS and service ceilings up to FL250, it represents one of the fastest certified piston singles ever produced.

This project aims to bring a JSBSim-based Mooney Bravo to FlightGear with progressively improved flight modeling, systems logic, cockpit functionality, and performance behavior.

---

## Project Status

### Alpha — active FDM and systems development

Current state:

- Aircraft structure created
- Aircraft loads successfully in FlightGear
- JSBSim FDM baseline implemented
- Metrics, mass balance, aerodynamics, flight controls, propulsion, ground reactions, and external reactions added
- Baseline property-binding and control-input issues resolved
- 3D model integrated
- Static FDM sanity checks completed
- Control-chain sanity checks completed
- Baseline retractable-gear FDM behavior implemented
- Constant-speed propeller behavior stabilized for climb testing
- Turbocharged high-altitude climb behavior tested up to FL250
- FL250 ceiling behavior validated with low remaining climb rate near service ceiling
- Pacejka-based tire model added for improved ground handling behavior
- Development progress is tracked through incremental Git commits

Partially implemented:

- Mooney-based geometry and reference values
- Early aerodynamic coefficient model
- Engine and propeller tuning outside the validated climb envelope
- Cruise, descent, and stall performance validation
- Retractable landing gear behavior
- Systems scaffolding
- External model and visual integration

Not yet implemented:

- Full POH-matched performance envelope
- Full cruise, descent, and stall validation
- Final engine / propeller tuning across all operating regimes
- Full retractable gear system animation and final polish
- Full electrical system
- Full fuel system
- Cockpit and instruments
- Finalized aircraft sounds
- Visual polish and detailed liveries

---

## Current Development Focus

The aircraft is currently in the FDM validation stage.

Recent work has focused on validating the turbocharged climb envelope and service-ceiling behavior. The aircraft has been tested up to FL250, where climb performance now correctly approaches service-ceiling behavior with only a small remaining climb rate.

The next planned development focus is cruise, descent, and stall-regression testing from the current validated climb baseline.

Ground handling is being developed separately from the airborne performance model. A Pacejka-based tire model has been added, but aero-side validation remains focused on JSBSim flight behavior.

---

## Goals

- Accurate Mooney M20M flight envelope
- Turbocharged Lycoming TIO-540 behavior
- Realistic constant-speed propeller behavior
- Realistic retractable gear logic
- Stable ground handling
- Full electrical and fuel systems
- Functional analog cockpit
- Cross-country performance matching against reference data where available
- Clean FlightGear aircraft structure suitable for long-term development

---

## Official Links

- Wikipedia  
  <https://en.wikipedia.org/wiki/Mooney_M20>

- Source Code Repository  
  <https://github.com/philip2012/Mooney-M20M>

- Issue Tracker / Support  
  <https://github.com/philip2012/Mooney-M20M/issues>

- FlightGear Wiki Page  
  <https://wiki.flightgear.org/Mooney_M20M_Bravo>

---

## Community

BravoWorks has launched a Discord server for development updates, discussion, and testing.

<https://discord.gg/tuMr5nbqps>

---

## Installation

Clone into your FlightGear `Aircraft` directory or any other aircraft directory loaded by FlightGear:

```bash
cd FlightGear/Aircraft
git clone https://github.com/philip2012/Mooney-M20M.git
```

---

## License

This project is licensed under the GNU General Public License v2.0 or later.
