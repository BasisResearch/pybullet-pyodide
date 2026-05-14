// Smoke test: load the cross-compiled pybullet wheel into Pyodide
// (running under Node) and exercise the core API.
//
// Run from this directory:  node smoke.mjs

import { loadPyodide } from "pyodide";
import { readFileSync, readdirSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

const __dirname = dirname(fileURLToPath(import.meta.url));
// pyodide build-recipes-no-deps drops the wheel here.
const DIST_DIR = resolve(__dirname, "../packages/pybullet/dist");

function findWheel() {
  const wheels = readdirSync(DIST_DIR).filter((f) =>
    f.startsWith("pybullet-") && f.endsWith(".whl"),
  );
  if (wheels.length !== 1) {
    throw new Error(
      `expected exactly one pybullet wheel in ${DIST_DIR}, found: ${wheels.join(", ") || "none"}`,
    );
  }
  return wheels[0];
}

async function main() {
  const wheelName = findWheel();
  const wheelPath = resolve(DIST_DIR, wheelName);

  console.log(`Loading Pyodide. Wheel: ${wheelName}`);
  const py = await loadPyodide();
  await py.loadPackage("numpy");

  py.FS.writeFile(`/tmp/${wheelName}`, readFileSync(wheelPath));
  await py.loadPackage("micropip");
  await py.runPythonAsync(`
import micropip
await micropip.install("emfs:/tmp/${wheelName}")
`);

  // The wheel is binding-only — pybullet_data ships no URDFs or meshes
  // (consumers mount their own at runtime). Stage minimal fixtures into
  // the in-Pyodide pybullet_data path before the smoke test reads them.
  const fixturesDir = resolve(__dirname, "fixtures");
  const dataPath = await py.runPythonAsync(
    `import pybullet_data; pybullet_data.getDataPath()`,
  );
  for (const fname of readdirSync(fixturesDir)) {
    py.FS.writeFile(`${dataPath}/${fname}`, readFileSync(`${fixturesDir}/${fname}`));
  }

  console.log("\n--- smoke test ---");
  const result = await py.runPythonAsync(`
import pybullet as p
import pybullet_data as pd
print("pybullet version:", getattr(p, "getAPIVersion", lambda: "?")())

cid = p.connect(p.DIRECT)
print("connected, client id:", cid)

p.setAdditionalSearchPath(pd.getDataPath())
p.setGravity(0, 0, -9.81)

plane_id = p.loadURDF("plane.urdf")
print("loaded plane, body id:", plane_id)

cube_id = p.loadURDF("cube_small.urdf", basePosition=[0, 0, 1.0])
print("loaded cube at z=1.0, body id:", cube_id)

for _ in range(240):
    p.stepSimulation()

pos, _ = p.getBasePositionAndOrientation(cube_id)
print(f"after 1s sim, cube z = {pos[2]:.4f} (should be near 0)")

p.disconnect()
print("disconnected")
"OK"
`);
  console.log("\nresult:", result);
}

main().catch((e) => {
  console.error("FAILED:", e);
  process.exit(1);
});
