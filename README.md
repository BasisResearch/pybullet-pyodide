# pybullet-pyodide

A [Pyodide](https://pyodide.org/) recipe for [pybullet](https://pybullet.org/)
— it cross-compiles upstream `pybullet` to WebAssembly so that the standard
`import pybullet as p` works inside a browser.

Headless physics only: URDF / MJCF / OBJ loading, rigid + soft body dynamics,
analytical IK, and `getCameraImage` via the CPU-only TinyRenderer. No GUI
(`p.connect(p.GUI)` will not work — use `p.connect(p.DIRECT)` and feed
`getCameraImage` output to Three.js or a canvas).


## Deviation from upstream pybullet
We choose to build the library with Emscripten's cmake wrapper with
scikit-build instead of reusing the upstream's `setup.py`. Based on
our past experience, cmake with release flag toggled generates faster
and smaller executables. In the case of pybullet, the resulting
executable is reduced from a few megabytes to less than 1 megabyte.


## Layout

```
packages/pybullet/
├── meta.yaml                # pyodide-recipes-style metadata
├── extras/
│   ├── CMakeLists.txt       # curated headless build (replaces upstream's)
│   └── pyproject.toml       # scikit-build-core build backend
└── patches/
    ├── 0001-drop-OpenGL-include-from-Wavefront2GLInstanceGraphicsShape.patch
    └── 0002-define-gSharedMemoryKey-locally.patch

test/
├── package.json             # pins pyodide@^0.29.4
├── fixtures/                # URDFs + OBJ meshes for the smoke test
└── smoke.mjs                # loads the wheel into Pyodide under Node and runs a sim
```

The recipe follows the conventions used in
[pyodide-recipes](https://github.com/pyodide/pyodide-recipes): upstream
source is fetched by URL + sha256, `source.extras` overlays files into
the unpacked tree (we overwrite upstream's `setup.py` / `CMakeLists.txt`
with a scikit-build-core + CMake pair that compiles only the headless
~80-file subset), and `source.patches` applies unified diffs (two small
diffs — one to drop an OpenGL include, one to define a global that
upstream only defined in the stripped GUI code).

## Build

The `pyodide` CLI lives in the `pyodide-cli` package; `pyodide-build`
plugs build subcommands into it. Install both:

The three version pins below — host Python, `pyodide-build`, and the
Pyodide runtime they target — must agree. See
[`.github/workflows/build.yml`](.github/workflows/build.yml) (under
`jobs.build_wheel.strategy.matrix`) for the combination CI is currently
exercising; substitute the same values here.

```bash
# pip is needed because pyodide-build shells out to it to install
# cross-build packages into the xbuildenv venv.
uv tool install --python <PYTHON> pyodide-cli \
    --with 'pyodide-build==<PYODIDE_BUILD>' --with pip

# `install` fetches the Pyodide sysroot; `install-emscripten` clones and
# patches the matching Emscripten SDK. Both are needed — don't BYO emcc:
# Pyodide pins a specific Emscripten version and applies side-module
# patches that aren't upstream (without them you'll see "bad export
# type" loader errors at wheel-import time).
pyodide xbuildenv install <PYODIDE_RUNTIME>
pyodide xbuildenv install-emscripten

source "$(pyodide config get emsdk_dir)/emsdk_env.sh"
pyodide build-recipes-no-deps pybullet --recipe-dir packages
```

Output:
`packages/pybullet/dist/pybullet-3.2.7-cp313-cp313-pyemscripten_2025_0_wasm32.whl`. The
Python and ABI tags follow whatever the installed xbuildenv targets
(0.29.4 = CPython 3.13, `pyemscripten_2025_0_wasm32`).

Use `build-recipes-no-deps`, not `build-recipes`. The plain `build-recipes`
command walks `requirements.host` / `requirements.run` and tries to build
every dependency from source — it expects a complete recipe tree like
[pyodide-recipes](https://github.com/pyodide/pyodide-recipes/), where
`numpy`, `openblas`, etc. all have their own `meta.yaml`. We only have
one recipe, so graph construction fails on `numpy`. The `-no-deps` driver
runs the exact same `meta.yaml` flow (fetch + sha256 + extras + patches
+ cross-compile) but skips dependency resolution. The deps are still
declared correctly in `meta.yaml`, so the recipe stays drop-in compatible
with `pyodide-recipes`.

## Smoke test

```bash
cd test/
npm install
node smoke.mjs
```

Loads the wheel from [`packages/pybullet/dist/`](packages/pybullet/dist/),
stages the URDF + mesh fixtures from [`test/fixtures/`](test/fixtures/)
into Pyodide's in-memory FS, drops a cube under gravity, and verifies
it lands near `z = 0`. Expected output:

```
pybullet build time: <date>
pybullet version: 202010061
connected, client id: 0
loaded plane, body id: 0
loaded cube at z=1.0, body id: 1
after 1s sim, cube z = 0.0250 (should be near 0)
disconnected
result: OK
```

## What's in the wheel

- Bullet core (LinearMath, Collision, Dynamics, SoftBody)
- The pybullet C binding (`pybullet.so`)
- SharedMemory in DIRECT (in-process) mode
- URDF / MJCF / Collada / OBJ importers
- TinyRenderer (CPU rasterizer for `getCameraImage`)
- BussIK analytical IK (powers `p.calculateInverseKinematics`)
- BulletInverseDynamics extras
- `pybullet_utils` (pure-Python helpers — `bullet_client`, `transformations`, etc.)
- `pybullet_data/__init__.py` only — **no bundled URDFs, meshes, or textures**.
  See [Asset loading](#asset-loading) below.

Explicitly excluded from the binary — none of these work or matter in
a browser:

- `p.connect(p.GUI)` (OpenGL / X11 / Cocoa / Win32 windows)
- ExampleBrowser
- Gwen GUI
- enet, clsocket (network shared-memory)
- EGL plugin
- VHACD
- Multi-threading (Bullet is built with `BT_THREADSAFE=0`)

## Asset loading

Pybullet's URDF/MJCF parser reads files (`*.urdf`, `*.obj`, `*.dae`,
textures) by name through C-level `fopen`. Under Pyodide that resolves
to Emscripten's in-memory MEMFS — there is no host filesystem to walk.
The wheel ships none of these assets, so anything you `loadURDF()` has
to be present in MEMFS first. A few patterns, lightest to heaviest:

**Inline.** Cheap, no network. Good for a handful of small files or
synthetic scenes:

```js
const URDF = `<?xml version="1.0"?><robot name="…">…</robot>`;
py.FS.writeFile("/assets/scene.urdf", URDF);
await py.runPythonAsync(`
  import pybullet as p
  p.connect(p.DIRECT)
  p.loadURDF("/assets/scene.urdf")     # absolute path, no search needed
`);
```

**Fetch a tarball / zip from a static host.** This is the production
pattern. Ship an `assets.tar.gz` next to the wheel on the same CDN and
unpack it into MEMFS in one shot:

```js
const buf = await (await fetch("https://cdn.example.com/assets.tar.gz")).arrayBuffer();
await py.unpackArchive(buf, "tar.gz", { extractDir: "/assets" });

await py.runPythonAsync(`
  import pybullet as p
  p.connect(p.DIRECT)
  p.setAdditionalSearchPath("/assets")  # now loadURDF("plane.urdf") works
  p.loadURDF("plane.urdf")
`);
```

Pyodide's `unpackArchive` handles `.tar`, `.tar.gz`, `.zip`, and a few
others. A compressed tarball of upstream's `pybullet_data` is ~22 MB —
the same size as the wheel itself, and worth lazy-loading or trimming
to just the URDFs/meshes your app actually uses.

**Mount the host filesystem (Node only).** During development, skip
the bundling and serve straight from disk via Emscripten's `NODEFS`:

```js
py.FS.mkdirTree("/assets");
py.FS.mount(py.FS.filesystems.NODEFS, { root: "./assets" }, "/assets");
```

**Persist across reloads in the browser.** `IDBFS` is the IndexedDB
backend — mount it once, populate it, then `FS.syncfs(false, …)` to
flush:

```js
py.FS.mkdirTree("/assets");
py.FS.mount(py.FS.filesystems.IDBFS, {}, "/assets");
await new Promise((ok, err) => py.FS.syncfs(true, e => e ? err(e) : ok()));
// ... populate /assets once ...
await new Promise((ok, err) => py.FS.syncfs(false, e => e ? err(e) : ok()));
```

Subsequent page loads see the populated `/assets` immediately after
the initial `syncfs(true, …)` — no re-download.

For any of these, the Python side is the same: either pass an absolute
path to `loadURDF`, or call `setAdditionalSearchPath` once and pass
short names. Pybullet resolves relative mesh references (`<mesh
filename="cube.obj"/>`) against the URDF's own directory first, then
falls back to the search paths.

## Upstream tracking

Everything compiled into the wheel comes from upstream
[`bulletphysics/bullet3`](https://github.com/bulletphysics/bullet3) (the
Python binding lives in `examples/pybullet/` of that repo and is
packaged to PyPI as `pybullet`). This recipe owns only the build
curation, not any C++ or Python code.

To bump the upstream version:

1. Update `version:` and `sha256:` in
   [`packages/pybullet/meta.yaml`](packages/pybullet/meta.yaml).
2. Re-run `pyodide build-recipes-no-deps`.
3. If the build fails:
   - Missing source file → upstream renamed or moved one of the paths
     in [`packages/pybullet/extras/CMakeLists.txt`](packages/pybullet/extras/CMakeLists.txt);
     find the new location and update it.
   - Undefined symbol at link → upstream added a new translation unit
     our binding depends on; add its path to the same
     [`CMakeLists.txt`](packages/pybullet/extras/CMakeLists.txt).
   - Header pulls in an OpenGL symbol → add a patch under
     [`packages/pybullet/patches/`](packages/pybullet/patches/)
     that swaps the offending `#include` for a narrower one.
   - `bad export type for ...: undefined` at runtime → upstream split
     a definition into a TU we don't compile; patch the using TU to
     either define the symbol locally or stop touching it.

Bullet moves slowly, so in practice this is light maintenance — but
it isn't zero, because both patches and the curated source list are
pinned to specific upstream paths.
