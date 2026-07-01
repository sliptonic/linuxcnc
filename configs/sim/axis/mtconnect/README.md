# MTConnect agent for LinuxCNC (prototype)

First-class MTConnect support for LinuxCNC: a userspace, non-realtime agent that
exposes machine **status**, a rich **kinematic description**, and **tool data**
over MTConnect. It ships an embedded HTTP agent (no external dependency) and can
optionally publish over the standard MTConnect **MQTT** binding. The `/probe`
response carries enough kinematic detail for an external tool (e.g. a FreeCAD
Path plugin) to auto-configure a machine.

This directory is a self-contained prototype that runs from a sim config. See
"Upstreaming" at the end for how it graduates into the main build.

## Quick start

```sh
# From this directory:
linuxcnc example.ini            # launches the sim + the MTConnect agent

# In another terminal:
curl http://localhost:5000/probe     # MTConnectDevices (structure + kinematics)
curl http://localhost:5000/current   # latest value of every DataItem
curl http://localhost:5000/sample?from=1&count=100
curl http://localhost:5000/assets    # CuttingTool assets (tool table)
```

You can inspect the generated device model without launching LinuxCNC:

```sh
./mtconnect-agent --dump-probe example.ini
./mtconnect-agent --dump-probe ../vismach/5axis/table-rotary-tilting/xyzac-trt.ini
```

## Enabling it in your own config

Add a few lines to your INI. The device model is generated automatically from
`[TRAJ]`, `[KINS]`, `[AXIS_*]` and `[JOINT_*]`.

```ini
[MTCONNECT]
ENABLE      = 1
DEVICE_NAME = my_mill
UUID        = linuxcnc-my-mill-0001
HTTP_PORT   = 5000
# TRANSPORT: http | mqtt | both  (no inline comments — LinuxCNC keeps the whole value)
TRANSPORT   = http
SAMPLE_HZ   = 10
# MQTT_BROKER = localhost
# MQTT_PORT   = 1883
# MQTT_PREFIX = MTConnect

[APPLICATIONS]
DELAY = 3
APP = /path/to/mtconnect-agent
```

`[APPLICATIONS]` starts the agent after the GUI, so all HAL pins exist. You also
need `HALUI = halui` in `[HAL]` (the agent uses it to detect shutdown).

## What is exposed

| Endpoint   | Content |
|------------|---------|
| `/probe`   | Device structure: Controller/Path, Axes (Linear/Rotary + Motion), spindle, and the `x:Kinematics` extension |
| `/current` | Latest value of every DataItem (execution, mode, positions, spindle, feed, tool) |
| `/sample`  | Sequence-numbered observation history (`?from=&count=`) |
| `/assets`  | Tool table as `CuttingTool` assets (location, diameter, length offsets) |

### LinuxCNC → MTConnect mapping (`linuxcnc.stat()`)

| MTConnect DataItem | Source |
|---|---|
| `EMERGENCY_STOP` | `task_state` |
| `CONTROLLER_MODE` | `task_mode` |
| `EXECUTION` | `interp_state`, `task_paused` |
| `PROGRAM`, `LINE_NUMBER` | `file`, `current_line` |
| `PATH_FEEDRATE` (+ OVERRIDE) | `current_vel`, `feedrate` |
| `POSITION` / `ANGLE` (ACTUAL/COMMANDED) | `actual_position`, `position` |
| `ROTARY_VELOCITY`, `ROTARY_MODE`, `DIRECTION` | `spindle[0]` |
| `TOOL_NUMBER`, `TOOL_ASSET_ID` | `tool_in_spindle` |
| `ASSET_CHANGED` | tool change detection |
| `CuttingTool` assets | `tool_table` |

## The kinematic description (`x:Kinematics`)

Standard MTConnect models axes as `Linear`/`Rotary` components with `Motion`
elements (`PRISMATIC`/`REVOLUTE`, direction vector). LinuxCNC specifics that do
not map cleanly are carried in a versioned extension namespace
`urn:linuxcnc:mtconnect:1` — the primary contract for auto-configuration:

```xml
<x:Kinematics module="xyzac-trt-kins" coordinates="XYZAC" joints="5"
              params="sparm=identityfirst">
  <x:JointMap>
    <x:Joint number="0" kind="LINEAR"  axis="X" min="-200" max="200" home="0"/>
    <x:Joint number="3" kind="ANGULAR" axis="A" min="-100" max="50"  home="0"/>
    <x:Joint number="4" kind="ANGULAR" axis="C" min="-36000" max="36000" home="0"/>
  </x:JointMap>
  <x:Axis name="X" kind="LINEAR"  vector="1 0 0" min="-200" max="200"/>
  <x:Axis name="A" kind="ANGULAR" vector="1 0 0" min="-100" max="50"/>
  <x:Axis name="C" kind="ANGULAR" vector="0 0 1" min="-36000" max="36000"/>
</x:Kinematics>
```

A FreeCAD plugin reads the standard `Axes`/`Motion` tree, or the compact
`x:Kinematics` block, to create/configure a machine: axis list and type,
travel limits, home positions, kinematics module/type, and the joint↔axis map.

## Digital twin (solid models)

An MTConnect twin (e.g. the viewer at demo.mtconnect.org/twin) renders a machine
from `/probe` alone: geometry is **referenced, never streamed**. The device model
carries `<SolidModel href="...">` elements pointing to external mesh files, plus
`<CoordinateSystems>` and a `<Motion>` chain; the viewer fetches each mesh once
and animates it using the streamed positions.

### Zero-config geometry (`MODEL_AUTO`)

```ini
[MTCONNECT]
MODEL_AUTO = 1
```

The agent generates simple placeholder **box** meshes for the base and each axis
straight from the travel limits and serves them from memory — any machine gets a
functional twin with **no mesh files**. This is what the example config uses.

### Real geometry (per-link meshes)

Supply your own meshes (STL / OBJ / glTF) for fidelity:

```ini
[MTCONNECT]
MODEL_DIR     = models        ; base dir for relative paths (default: INI dir)
MODEL_UNITS   = MILLIMETER    ; the MESH's units (not the machine's)
MODEL_BASE    = frame.stl     ; static frame / column (device-level SolidModel)
MODEL_X       = x_table.stl   ; link that moves with X
MODEL_Y       = y_saddle.stl
MODEL_Z       = spindle.stl
MODEL_SPINDLE = spindle.stl
```

Author each mesh in the machine frame at the all-axes-zero pose. An explicit
`MODEL_<axis>` overrides the generated box for that link, so files and
`MODEL_AUTO` can be mixed.

### Topology and direction (both modes)

These describe the mechanics and are **not** derivable from a trivkins INI:

```ini
MODEL_CHAIN    = Y X Z    ; nesting order of the moving links (base -> tip)
MODEL_PARENT_Z = BASE     ; branch: Z (quill/tool) hangs off the base, not X/Y
MODEL_INVERT   = X Y      ; work-carrying links move opposite the reported coord
```

The agent then serves each mesh at `GET /models/<name>`, emits a device-level
`<SolidModel>` for the base and a per-axis `<SolidModel>` in each axis's
`<Configuration>`, and emits `<CoordinateSystems>` (WORLD → MACHINE) plus a
`<Motion>` chain (`parentIdRef`) so the twin nests transforms correctly.

## Using the official cppagent instead of the embedded agent

The generated `/probe` document doubles as a cppagent `Devices.xml`:

```sh
./mtconnect-agent --dump-probe example.ini > Devices.xml
```

A future `TRANSPORT = shdr` mode will stream the same DataItems over SHDR to a
cppagent configured with this `Devices.xml`, proving the model is
standards-portable. (SHDR output is not implemented in this prototype.)

## MQTT

With `TRANSPORT = mqtt` (or `both`) and `python3-paho-mqtt` installed, the agent
publishes to the standard MTConnect MQTT topics:
`<prefix>/Probe/<uuid>` (retained), `<prefix>/Current/<uuid>`,
`<prefix>/Sample/<uuid>`, `<prefix>/Asset/<uuid>/<assetId>`.

## Layout

```
mtconnect-agent        entry point: INI + HAL component + poll loop + transports
mtc/ini_reader.py      INI access (linuxcnc.ini, with offline fallback)
mtc/kinematics.py      build the kinematic model from the INI
mtc/observations.py    shared DataItem registry (keeps probe and streams in sync)
mtc/device_model.py    /probe MTConnectDevices document (+ CLI --dump-probe)
mtc/lcnc_source.py     live status + tool table from linuxcnc.stat()
mtc/models.py          solid-model config + served mesh registry (digital twin)
mtc/streams.py         /current, /sample buffer + XML; /assets XML
mtc/agent.py           transport-agnostic agent core (document builders)
mtc/http_agent.py      embedded HTTP server
mtc/mqtt_agent.py      optional MQTT publisher
test_mtc.py            offline test suite (no running LinuxCNC required)
```

## Testing

```sh
python3 test_mtc.py
```

Runs fully offline: kinematics mapping, probe↔registry consistency (3- and
5-axis), the observation buffer + streams, assets, and the live HTTP endpoints.

## Upstreaming (deferred)

To graduate from prototype to a merge-ready feature:

1. Move `mtconnect-agent` and `mtc/` into `src/hal/user_comps/`; add the entry
   stem to `USER_COMP_PY` in that directory's `Submakefile` (installs to `bin/`
   and flows into the `linuxcnc-uspace` Debian package).
2. Add `docs/src/man/man1/mtconnect-agent.1.adoc` (auto-discovered by the man
   glob), modeled on `mqtt-publisher.1.adoc`.
3. Add an integrator chapter `docs/src/config/mtconnect.adoc` and document the
   `[MTCONNECT]` section in `docs/src/config/ini-config.adoc`.
4. Add `python3-paho-mqtt` as a `Recommends` if MQTT is kept.
5. Optional: a thin C/HAL pin-bridge for signals not present in `linuxcnc.stat()`.
