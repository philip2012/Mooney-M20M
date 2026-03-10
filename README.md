# Mooney M20M Bravo — FlightGear

Work-in-progress Mooney M20M Bravo for FlightGear.

The Mooney M20M Bravo is a high-performance, turbocharged, retractable-gear four-seat piston aircraft from Mooney’s long-body M20 family. With cruise speeds exceeding 200 KTAS and service ceilings up to FL250, it represents one of the fastest certified piston singles ever produced.

This project aims to bring a realistic, JSBSim-based Mooney Bravo to FlightGear with accurate performance, systems, and cockpit modeling.

---

## Project Status

### Alpha — early development

Current state:

- Aircraft structure created
- JSBSim FDM baseline implemented
- Metrics, mass balance, aerodynamics, ground reactions, external reactions, flight controls, and starter propulsion stack added
- Aircraft loads successfully in FlightGear
- Baseline property-binding and control-input issues resolved
- Placeholder model support
- Static FDM sanity checks completed
- Control-chain sanity checks completed

Partially implemented:

- Mooney-based geometry and reference values
- Baseline retractable-gear FDM behavior
- Early aerodynamic coefficient model

Not yet implemented:

- Runtime-validated and tuned engine / propeller behavior
- Full retractable gear system and animation
- Electrical / fuel systems
- Cockpit and instruments
- Full performance tuning and POH matching

---

## Goals

- Accurate Mooney M20M flight envelope
- Turbocharged Lycoming TIO-540 engine behavior
- Realistic retractable gear logic
- Full electrical and fuel systems
- Functional analog cockpit
- Cross-country performance matching POH data

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

## Installation

Clone into your FlightGear `Aircraft` directory or any desired place：

```bash
cd FlightGear/Aircraft
git clone https://github.com/philip2012/Mooney-M20M.git
```
