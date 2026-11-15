# Contributing to the Mooney M20M Bravo

Thanks for helping improve the Mooney M20M Bravo for [FlightGear](https://www.flightgear.org/). This aircraft is a work in progress, using JSBSim for its flight dynamics. Contributions of code, models, documentation, reference data, bug reports, and reproducible test results are welcome.

Before starting a substantial change, check the [README](README.md) for the current development status and browse the [issue tracker](https://github.com/philip2012/Mooney-M20M/issues) to avoid duplicating work.

## Artificial intelligence

**AI-assisted development is allowed.** You may use AI/LLMs for code, XML, Nasal, documentation, research assistance, modeling workflows, or any other part of your contribution.

- AI-assisted and manually produced contributions are evaluated against the **same technical criteria**.
- You do **not** need to disclose AI usage, estimate how much AI was used, or certify that you manually reviewed or understood every line.
- Acceptance is based on the resulting contribution: correct behavior, compatibility with the aircraft, appropriate evidence, maintainability, and licensing/provenance.

However, we encourage contributors to specify the model, reasoning effort, and vendor of the AI that they used for contributing. This gives us a clearer picture of that specific Issue/PR, for example: "I used OpenAI's GPT-6 Astra at High effort".

No authoring method is automatically grounds for acceptance or rejection.

## Where changes belong

Keep responsibilities separate so that changes can be tested and maintained independently.

| Layer | Location | Responsibility |
| --- | --- | --- |
| Aircraft configuration | `Mooney-M20M-set.xml` | FlightGear discovery, startup configuration, metadata, and links to aircraft components |
| Flight dynamics | `FDM/`, `Engines/` | JSBSim geometry, mass/inertia, propulsion, aerodynamics, and ground reactions |
| Model | `Models/`, `Blender/` | Geometry, model wrappers, animations, textures, and effects |
| Systems | `Systems/` | Aircraft systems and property integration |
| Aircraft states | `States/` | Initial conditions and startup presets |
| Scripting | `Nasal/` | Aircraft-specific scripting where needed |
| Development material | `development/` | Development and validation resources, not automatically production aircraft logic |

Avoid mixing unrelated FDM tuning, model changes, systems behavior, and formatting into one pull request. Use repository-relative paths where possible; verify any FlightGear-resolved paths and XML includes before changing them.

## What to validate

**Use the checks relevant to your change**, not an unrelated checklist. A documentation correction does not need a flight test; a JSBSim change usually does.

- **Aircraft configuration or paths:** Confirm the aircraft is discoverable and loads in the target FlightGear version. Check referenced files, XML validity, include resolution, and launcher metadata when affected.
- **JSBSim, engines, and flight controls:** Confirm the FDM loads, relevant controls or outputs behave as intended, and the change does not regress the affected operating conditions. Support real-aircraft specifications and performance claims with references where available; identify estimated values as estimates.
- **Model, animations, and effects:** Confirm model loading, placement/orientation, the relevant property bindings, and the changed animation or effect in FlightGear. Account for renderer or FlightGear-version differences where relevant.
- **Systems, states, and Nasal:** Check the relevant property inputs/outputs, initialization, and affected startup states. Avoid accidental coupling between independent systems.
- **Multiplayer or input changes:** Check the affected multiplayer-visible properties or input bindings rather than assuming local behavior is sufficient.

If a check cannot be performed, say which check is missing and why. Do not claim a behavior has been verified without evidence.

## Pull requests

Fork the repository, make a focused change, and open a pull request against the current development branch (normally `main`). A useful PR description contains:

1. **Purpose:** What problem is fixed or what capability is added.
2. **Scope:** Which aircraft layer(s) and files changed.
3. **Evidence:** Relevant test commands/results, reproduction steps, screenshots, FlightGear version, or flight conditions, as appropriate.
4. **References:** Sources for new aircraft dimensions, performance targets, or other real-world specifications, if applicable.

Prefer small, reviewable commits. Keep structural or path changes separate from flight-dynamics tuning whenever practical. Changes that break aircraft discovery, model or JSBSim loading, property bindings, startup states, or cross-platform paths may be declined until resolved.

## Licensing, assets, and references

This project is distributed under the **GNU General Public License version 2 or later**; see [COPYING](COPYING) and [LICENSE](LICENSE).

- Preserve existing copyright and license notices, and provide appropriate attribution for third-party work.
- Record new third-party asset/code origins and licensing in [PROVENANCE.md](PROVENANCE.md) when applicable.
- Do not commit copyrighted manual scans, drawings, or other reference material without distribution rights. Factual measurements and specifications may be used as research inputs, with sources identified where practical.
- Follow the repository's Git LFS setup for large binary assets and source models. Avoid unnecessary generated files or unrelated binary changes.

Questions, proposals, and reproducible bugs can be raised through [GitHub Issues](https://github.com/philip2012/Mooney-M20M/issues).
