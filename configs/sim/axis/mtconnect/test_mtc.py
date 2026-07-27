#!/usr/bin/env python3
# Offline tests for the MTConnect agent prototype.
#
# Exercises the probe generator, the probe<->registry consistency, the
# observation buffer + streams, and the asset builder without a running
# LinuxCNC.  Run:  python3 test_mtc.py
import subprocess
import sys
import urllib.request
import xml.etree.ElementTree as ET

from mtc.ini_reader import IniReader
from mtc import kinematics as kin
from mtc.device_model import DeviceConfig, probe_xml, MTC_NS, EXT_NS
from mtc.observations import build_dataitems
from mtc.streams import (ObservationBuffer, current_xml, sample_xml, assets_xml,
                         STREAMS_NS)
from mtc.lcnc_source import ToolAsset, LcncSource
from mtc.agent import AgentState
from mtc.http_agent import HttpAgent

CONFIGS = {
    "3axis": "../axis.ini",
    "5axis": "../vismach/5axis/table-rotary-tilting/xyzac-trt.ini",
}
TS = "2026-07-01T12:00:00.000Z"


def load(path):
    ini = IniReader(path)
    model = kin.build_model(ini)
    config = DeviceConfig.from_ini(ini)
    return ini, model, config


def test_kinematics():
    _, model, _ = load(CONFIGS["5axis"])
    assert model.kins_module == "xyzac-trt-kins", model.kins_module
    assert model.coordinates == "XYZAC", model.coordinates
    assert model.joints_count == 5
    jmap = model.joint_axis_map()
    assert jmap == {0: "X", 1: "Y", 2: "Z", 3: "A", 4: "C"}, jmap
    kinds = {a.letter: a.kind for a in model.axes}
    assert kinds["A"] == "ANGULAR" and kinds["X"] == "LINEAR", kinds
    # A rotates about X, C about Z
    vecs = {a.letter: a.vector for a in model.axes}
    assert vecs["A"] == (1.0, 0.0, 0.0) and vecs["C"] == (0.0, 0.0, 1.0), vecs
    print("ok  kinematics (5-axis XYZAC mapping + rotary vectors)")


def test_probe_registry_consistency():
    for label, path in CONFIGS.items():
        _, model, config = load(path)
        xml = probe_xml(model, config, creation_time=TS)
        root = ET.fromstring(xml)
        probe_ids = {}
        for di in root.iter("{%s}DataItem" % MTC_NS):
            probe_ids[di.get("id")] = (di.get("category"), di.get("type"),
                                       di.get("subType"))
        for d in build_dataitems(model, config):
            assert d.id in probe_ids, "%s: %s missing from probe" % (label, d.id)
            cat, typ, sub = probe_ids[d.id]
            expected_type = ("x:" + d.type) if d.ext else d.type
            assert (cat, typ, sub) == (d.category, expected_type, d.subType), \
                "%s: %s attrs %s != %s" % (label, d.id, (cat, typ, sub),
                                           (d.category, d.type, d.subType))
        # kinematics extension present
        kext = root.find(".//{%s}Kinematics" % EXT_NS)
        assert kext is not None, "%s: no kinematics extension" % label
        assert kext.get("module") == model.kins_module
        print("ok  probe/registry consistency (%s, %d DataItems)"
              % (label, len(probe_ids)))


def test_buffer_and_streams():
    _, model, config = load(CONFIGS["3axis"])
    dataitems = build_dataitems(model, config)
    buf = ObservationBuffer(dataitems)

    n = buf.ingest({"execution": "READY", "pos_x": 1.0, "pos_y": 2.0}, TS)
    assert n == 3, n
    assert buf.ingest({"pos_x": 1.0}, TS) == 0, "unchanged value must not record"
    assert buf.ingest({"pos_x": 5.0}, TS) == 1
    assert buf.last_sequence == 4, buf.last_sequence

    # /current: every DataItem present, unset ones UNAVAILABLE
    cur = ET.fromstring(current_xml(buf, config, TS))
    observed = {e.get("dataItemId"): e.text
                for e in cur.iter() if e.get("dataItemId")}
    assert observed["pos_x"] == "5.0", observed.get("pos_x")
    assert observed["pos_z"] == "UNAVAILABLE", observed.get("pos_z")
    assert observed["execution"] == "READY"

    # /sample: sequence range 1..2 returns exactly the first two observations
    smp = ET.fromstring(sample_xml(buf, config, TS, from_seq=1, count=2))
    seqs = sorted(int(e.get("sequence")) for e in smp.iter()
                  if e.get("sequence"))
    assert seqs == [1, 2], seqs
    # ComponentStream grouping + Samples/Events wrappers exist
    assert cur.find(".//{%s}ComponentStream" % STREAMS_NS) is not None
    print("ok  buffer + current/sample streams")


def test_work_offset_table():
    _, model, config = load(CONFIGS["3axis"])
    buf = ObservationBuffer(build_dataitems(model, config))
    buf.ingest({"workoffset": {"G55": {"X": 1.5, "Y": -2.0, "Z": 0.0}}}, TS)
    cur = ET.fromstring(current_xml(buf, config, TS))
    wo = cur.find(".//{%s}WorkOffset" % STREAMS_NS)
    assert wo is not None and wo.get("count") == "1", wo
    entry = wo.find("{%s}Entry" % STREAMS_NS)
    assert entry.get("key") == "G55", entry.attrib
    cells = {c.get("key"): c.text for c in entry.findall("{%s}Cell" % STREAMS_NS)}
    assert cells == {"X": "1.5", "Y": "-2", "Z": "0"}, cells
    # probe advertises it as a TABLE representation
    probe = ET.fromstring(probe_xml(model, config))
    di = [d for d in probe.iter("{%s}DataItem" % MTC_NS)
          if d.get("id") == "workoffset"][0]
    assert di.get("representation") == "TABLE" and di.get("type") == "WORK_OFFSET"
    print("ok  work offset TABLE (G5x/G92 Entry/Cell + probe representation)")


def test_tool_offset_and_rotation():
    _, model, config = load(CONFIGS["3axis"])
    buf = ObservationBuffer(build_dataitems(model, config))
    buf.ingest({"tooloffset": {"T3": {"X": 0.0, "Y": 0.0, "Z": 2.5}},
                "xyrotation": 30.0}, TS)
    cur = ET.fromstring(current_xml(buf, config, TS))

    # extension elements live in the LinuxCNC namespace
    to = cur.find(".//{%s}ToolOffset" % EXT_NS)
    assert to is not None, "no x:ToolOffset in stream"
    entry = to.find("{%s}Entry" % EXT_NS)
    assert entry.get("key") == "T3"
    zc = [c for c in entry.findall("{%s}Cell" % EXT_NS) if c.get("key") == "Z"][0]
    assert zc.text == "2.5", zc.text
    rot = cur.find(".//{%s}CoordinateRotation" % EXT_NS)
    assert rot is not None and rot.text == "30.0", rot

    # probe advertises extension types with the x: prefix
    probe = ET.fromstring(probe_xml(model, config))
    types = {d.get("id"): d.get("type") for d in probe.iter("{%s}DataItem" % MTC_NS)}
    assert types["tooloffset"] == "x:TOOL_OFFSET", types.get("tooloffset")
    assert types["xyrotation"] == "x:COORDINATE_ROTATION", types.get("xyrotation")
    print("ok  tool offset + XY rotation extensions (x: namespace)")


def test_assets():
    _, _, config = load(CONFIGS["3axis"])
    tools = [
        ToolAsset(tool_no=1, pocket=1, in_spindle=True, diameter=6.35, length_z=50.0,
                  comment="1/4 flat endmill"),
        ToolAsset(tool_no=2, pocket=5, in_spindle=False, diameter=3.0),
    ]
    root = ET.fromstring(assets_xml(tools, config, TS))
    ns = "{urn:mtconnect.org:MTConnectAssets:1.7}"
    cts = root.findall(".//%sCuttingTool" % ns)
    assert len(cts) == 2, len(cts)
    assert cts[0].get("assetId") == "tool-1"
    loc = cts[0].find(".//%sLocation" % ns)
    assert loc.get("type") == "SPINDLE" and loc.text == "1", loc.attrib
    desc = cts[0].find("%sDescription" % ns)
    assert desc is not None and desc.text == "1/4 flat endmill", ET.tostring(cts[0])
    assert cts[1].find("%sDescription" % ns) is None  # empty comment -> no element
    print("ok  assets (CuttingTool location + measurements + comment)")


def test_source_offline():
    _, model, config = load(CONFIGS["3axis"])
    src = LcncSource(model, config)
    # No linuxcnc extension / no running instance -> graceful empty results.
    if not src.available():
        assert src.sample_values() == {}
        assert src.tool_assets() == []
        print("ok  lcnc_source offline (graceful, no stat)")
    else:
        print("ok  lcnc_source live (stat available)")


def test_http_endpoints():
    state = AgentState(CONFIGS["3axis"])
    state.buffer.ingest({"execution": "ACTIVE", "pos_x": 3.5}, TS)
    state._assets = [ToolAsset(tool_no=7, pocket=7, in_spindle=True, diameter=10.0)]

    http = HttpAgent(state, host="127.0.0.1", port=0)
    http.start()
    try:
        base = "http://127.0.0.1:%d" % http.port

        def get(path):
            with urllib.request.urlopen(base + path, timeout=5) as r:
                return r.status, r.read().decode()

        st, body = get("/probe")
        assert st == 200 and "MTConnectDevices" in body, st
        st, body = get("/current")
        assert st == 200 and "ACTIVE" in body, body[:200]
        st, body = get("/sample?from=1&count=10")
        assert st == 200 and 'sequence="1"' in body, body[:200]
        st, body = get("/assets")
        assert st == 200 and "tool-7" in body, body[:200]

        # Bad route and out-of-range sequence return MTConnectError docs.
        try:
            get("/nope")
            assert False, "expected 404"
        except urllib.error.HTTPError as e:
            assert e.code == 404 and "MTConnectError" in e.read().decode()
        try:
            get("/sample?from=999999")
            assert False, "expected 406"
        except urllib.error.HTTPError as e:
            assert e.code == 406 and "OUT_OF_RANGE" in e.read().decode()
        print("ok  embedded HTTP agent (probe/current/sample/assets + errors)")
    finally:
        http.stop()


def test_solid_models():
    import os
    import tempfile
    from mtc.models import MachineModels, MeshRef

    _, model, config = load(CONFIGS["3axis"])

    # No models configured -> no SolidModel / Configuration emitted.
    plain = ET.fromstring(probe_xml(model, config))
    assert plain.find(".//{%s}SolidModel" % MTC_NS) is None

    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, "z_head.stl")
    with open(path, "wb") as fh:
        fh.write(b"solid dummy\nendsolid dummy\n")
    ref = MeshRef("z_head.stl", path, "STL", "model/stl", True)
    mm = MachineModels(units="MILLIMETER", base=ref, axis={"Z": ref},
                       chain=["dev_base", "axis_x", "axis_y", "axis_z"],
                       parents={"axis_x": "dev_base", "axis_y": "axis_x",
                                "axis_z": "axis_y"},
                       served={"z_head.stl": ref})

    probe = ET.fromstring(probe_xml(model, config, models=mm))
    sms = probe.findall(".//{%s}SolidModel" % MTC_NS)
    hrefs = {sm.get("href") for sm in sms}
    assert "/models/z_head.stl" in hrefs, hrefs
    assert probe.find(".//{%s}CoordinateSystem[@type='MACHINE']" % MTC_NS) is not None
    # serial chain: Z link hangs off Y's motion
    mz = [m for m in probe.iter("{%s}Motion" % MTC_NS) if m.get("id") == "motion_z"][0]
    assert mz.get("parentIdRef") == "motion_y", mz.attrib

    # branched: re-root Z at the base (knee-mill quill) -> no parentIdRef
    mm.parents["axis_z"] = "dev_base"
    probe2 = ET.fromstring(probe_xml(model, config, models=mm))
    mz2 = [m for m in probe2.iter("{%s}Motion" % MTC_NS) if m.get("id") == "motion_z"][0]
    assert mz2.get("parentIdRef") is None, mz2.attrib

    # inverted axis: Motion <Axis> vector is negated (work-carrying link)
    mm.invert = {"Z"}
    probe3 = ET.fromstring(probe_xml(model, config, models=mm))
    mz3 = [m for m in probe3.iter("{%s}Motion" % MTC_NS) if m.get("id") == "motion_z"][0]
    assert mz3.find("{%s}Axis" % MTC_NS).text == "0 0 -1", mz3.find("{%s}Axis" % MTC_NS).text

    # /models route serves the file; unknown model -> 404
    state = AgentState(CONFIGS["3axis"])
    state.models = mm
    http = HttpAgent(state, host="127.0.0.1", port=0)
    http.start()
    try:
        with urllib.request.urlopen(
                "http://127.0.0.1:%d/models/z_head.stl" % http.port, timeout=5) as r:
            assert r.status == 200 and b"endsolid" in r.read()
        try:
            urllib.request.urlopen("http://127.0.0.1:%d/models/nope.stl" % http.port)
            assert False, "expected 404"
        except urllib.error.HTTPError as e:
            assert e.code == 404
        print("ok  solid models (SolidModel/CoordinateSystems/chain + /models route)")
    finally:
        http.stop()


def test_auto_geometry():
    import os
    import tempfile
    from mtc.models import build_models

    ini_text = (
        "[EMC]\nMACHINE = auto-demo\n"
        "[TRAJ]\nCOORDINATES = X Y Z\nLINEAR_UNITS = inch\n"
        "[KINS]\nKINEMATICS = trivkins\nJOINTS = 3\n"
        "[AXIS_X]\nMIN_LIMIT = -10\nMAX_LIMIT = 10\n"
        "[AXIS_Y]\nMIN_LIMIT = -6\nMAX_LIMIT = 6\n"
        "[AXIS_Z]\nMIN_LIMIT = -8\nMAX_LIMIT = 0\n"
        "[MTCONNECT]\nMODEL_AUTO = 1\nMODEL_PARENT_Z = BASE\nMODEL_INVERT = X Y\n"
    )
    d = tempfile.mkdtemp()
    p = os.path.join(d, "auto.ini")
    with open(p, "w") as fh:
        fh.write(ini_text)

    ini = IniReader(p)
    model = kin.build_model(ini)
    config = DeviceConfig.from_ini(ini)
    mm = build_models(ini, model, config)
    assert mm.enabled() and mm.base is not None
    assert set(mm.axis) == {"X", "Y", "Z"}, set(mm.axis)
    assert mm.units == "INCH", mm.units  # auto meshes use machine units
    assert all(v.startswith("solid") for v in mm.generated.values())

    probe = ET.fromstring(probe_xml(model, config, models=mm))
    hrefs = {sm.get("href") for sm in probe.findall(".//{%s}SolidModel" % MTC_NS)}
    assert {"/models/dev_base.stl", "/models/axis_x.stl", "/models/axis_z.stl"} <= hrefs, hrefs

    # the agent serves the generated STL bytes (no file on disk)
    state = AgentState(p)
    http = HttpAgent(state, host="127.0.0.1", port=0)
    http.start()
    try:
        with urllib.request.urlopen(
                "http://127.0.0.1:%d/models/axis_x.stl" % http.port, timeout=5) as r:
            assert r.status == 200 and b"endsolid" in r.read()
        print("ok  auto geometry (generated boxes served + probe SolidModels)")
    finally:
        http.stop()


def test_ha_discovery():
    import json
    from mtc import ha

    _, model, config = load(CONFIGS["3axis"])
    sensors = ha.build_sensors(model, config)
    keys = {s["key"] for s in sensors}
    assert {"execution", "spdl_speed", "pos_x", "pos_y", "pos_z"} <= keys, keys

    px = [s for s in sensors if s["key"] == "pos_x"][0]
    d = ha.discovery_payload(px, config, "st/topic", "av/topic")
    assert d["state_topic"] == "st/topic"
    assert d["value_template"] == "{{ value_json.pos_x }}"
    assert d["unit_of_measurement"] == "in"          # inch machine
    assert d["device"]["identifiers"] == [config.uuid]
    assert d["unique_id"] == "%s_pos_x" % config.uuid

    state = json.loads(ha.state_json(
        {"execution": "ACTIVE", "pos_x": 1.5, "pos_z": "UNAVAILABLE",
         "workoffset": {"G54": {}}}, sensors))
    assert state["execution"] == "ACTIVE" and state["pos_x"] == 1.5
    assert "pos_z" not in state          # UNAVAILABLE skipped
    assert "workoffset" not in state     # structured value skipped
    print("ok  HA discovery (sensors + discovery payload + state JSON)")


def test_entry_dump_probe():
    out = subprocess.check_output(
        [sys.executable, "./mtconnect-agent", "--dump-probe", CONFIGS["5axis"]],
        text=True)
    assert "xyzac-trt-kins" in out and "MTConnectDevices" in out
    print("ok  entry --dump-probe")


def main():
    test_kinematics()
    test_probe_registry_consistency()
    test_buffer_and_streams()
    test_work_offset_table()
    test_tool_offset_and_rotation()
    test_assets()
    test_source_offline()
    test_http_endpoints()
    test_solid_models()
    test_auto_geometry()
    test_ha_discovery()
    test_entry_dump_probe()
    print("\nALL TESTS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
