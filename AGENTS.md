# Repository Guidelines

## Project Structure & Module Organization
- `apps/` stores showcase demos such as `popcorn`; each includes its own `CMakeLists.txt` and assets.
- `audio/`, `scanvideo/`, `sleep/`, `reset/`, and `stdio/` expose subsystems; add new examples alongside the closest match.
- `standalone/` supplies minimal templates for fresh experiments without scaffolding.
- `tools/` holds helper utilities; keep generated binaries next to their sources.
- `build/` is disposable output; leave it untracked and create `build/<board>` variants when switching hardware.

## Build, Test, and Development Commands
- `export PICO_SDK_PATH=/path/to/pico-sdk` and `export PICO_EXTRAS_PATH=/path/to/pico-extras` before configuring.
- `cmake -S . -B build -DPICO_PLATFORM=rp2040` sets up the tree; swap the platform flag (e.g., `rp2350`) per target board.
- `cmake --build build --target popcorn -j` compiles a single demo; run `ninja -C build` to rebuild everything.
- `picotool load build/apps/popcorn/popcorn.uf2 --family RP2040 --execute` flashes a board; update the UF2 path to match your module.

## Coding Style & Naming Conventions
- Code is C11 with some C++17; mirror the Pico SDK style: tab indentation, K&R braces, trailing commas in initializer lists.
- Use `snake_case` for functions and variables, reserve `CamelCase` for types, and keep macros in `UPPER_SNAKE_CASE`.
- Order includes SDK → project → local, and keep CMake options in the nearest `CMakeLists.txt` to maintain modular builds.
- Run `clang-format` with the Pico profile when available; otherwise match the surrounding file’s layout and spacing.

## Testing Guidelines
- Automated tests are not wired; build the affected target and exercise it on hardware before requesting review.
- Record wiring, board definitions, or clock tweaks in the module README so others can reproduce your setup.
- Add a lightweight smoke check (serial banner, frame counter, assertion) to new demos to aid manual testing.

## Commit & Pull Request Guidelines
- Write commit subjects in imperative mood (`Fix USB clock calc`), keep them concise, and append `(#issue)` when closing tickets.
- Squash noisy WIP commits locally so the history reflects logical milestones (setup, driver, docs).
- PRs should list target board(s), verification steps, and host tooling; attach screenshots or logs for visual or audio changes.

## Hardware & Configuration Tips
- Set `PICO_BOARD`, `PICO_PLATFORM`, and overclock tweaks via CMake cache variables, and keep separate `build/<user>-<board>` directories to avoid clobbering artefacts while switching targets.
