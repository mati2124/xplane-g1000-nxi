# AGENTS.md

## Cursor Cloud specific instructions

This is a C++17 / CMake project: the **X-Plane G1000 NXi** glass-cockpit avionics
suite (`avionics-core` engine + `shell-standalone` desktop app + optional
`shell-xplane` plugin). See `README.md` and `ARCHITECTURE.md` for the full
picture; only non-obvious cloud caveats are recorded here.

### Compiler: use gcc/g++, not the default `c++`
On this image the `c++`/`cc` alternatives point at **clang 18**, whose linker
fails with `cannot find -lstdc++`. Always configure CMake with GCC:

```bash
cmake -S . -B build -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ \
  -DCMAKE_BUILD_TYPE=Debug -DBUILD_STANDALONE_SHELL=ON -DBUILD_TESTS=ON
cmake --build build -j"$(nproc)"
```

The first configure downloads GLFW, NanoVG, nlohmann/json, stb and GoogleTest via
CMake `FetchContent`, so **network is required on the first configure**.

### Lint / Test / Build / Run
- **Tests** (pure CPU, no display): `cd build && ctest --output-on-failure`
  (target `avionics-core-tests`, 90 GoogleTest cases). Built only when
  `-DBUILD_TESTS=ON`.
- **Build**: `cmake --build build -j"$(nproc)"` (default target builds
  `avionics-core` + `avionics-standalone`).
- **No formal lint target / no pre-commit hooks** exist in this repo.
- **X-Plane plugin** (`-DBUILD_XPLANE_SHELL=ON`) needs the X-Plane SDK via
  `-DXPLANE_SDK_DIR=...` and a real X-Plane sim to run — not runnable headless.

### Running the standalone app headless
The standalone shell uses **GLFW + OpenGL over X11** even for its offscreen
`--screenshot` mode (it still calls `glfwInit()` and makes a hidden GL context),
so it needs a virtual display + software GL. Run it under `xvfb-run` with Mesa
llvmpipe:

```bash
export AVIONICS_SKIP_UPDATE_CHECK=1 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
xvfb-run -a -s "-screen 0 1280x1024x24" \
  ./build/shell-standalone/avionics-standalone \
  --screenshot out.ppm --state mfd --no-bezel
```

- `--screenshot` renders deterministic frames from a built-in `MockDataSource`,
  so it works with **no X-Plane running**. Useful `--state` values include
  `pfd`, `mfd`, `map`, `boot` (see `shell-standalone/src/main.cpp` for the full
  list).
- Without `--screenshot` the app opens live PFD/MFD windows and connects to
  X-Plane over UDP (default `127.0.0.1:49000`); with no sim it just shows the
  red-X "no data" annunciation.
- Output is **PPM**; there is no image converter preinstalled — convert with a
  small Python/`zlib` script or install one if you need PNG.
- Always set `AVIONICS_SKIP_UPDATE_CHECK=1` in CI/cloud to skip the GitHub
  Releases update check.

### System dependencies (already provided by the update script / image)
`libcurl4-openssl-dev libglew-dev xorg-dev libgl1-mesa-dev` plus Mesa software
GL (`libgl1-mesa-dri`) and `xvfb`. `python3` is used by `tools/fetch_obstacles.py`.

### Required secrets (Navigraph / SimBrief OFP import)
The SimBrief import authenticates against Navigraph via the OAuth device flow.
`NavigraphClient::LoadNavigraphCredentials` reads the OAuth client credentials,
in this precedence order:

1. environment variables (preferred — use these for Cloud Agents)
2. a git-ignored `navigraph_credentials.txt` (repo root, then per-user config dir)

Required environment-variable **names** (set the values in the Cursor dashboard
**Secrets** tab at <https://cursor.com/dashboard/cloud-agents>, as Runtime
Secrets — `environment.json` has no field for secret names/values, so never
commit them there or anywhere in git):

- `NAVIGRAPH_CLIENT_ID` — Navigraph OAuth client id
- `NAVIGRAPH_CLIENT_SECRET` — Navigraph OAuth client secret

Note: Navigraph traffic is gated to a connected simulator session (their terms).
The in-sim plugin is always permitted; the standalone shell only talks to
Navigraph while it has a live X-Plane link, so interactive sign-in / OFP fetch
do **not** work in headless `--screenshot` runs.
