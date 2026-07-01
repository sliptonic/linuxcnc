# Build the MTConnectDevices (/probe) document for a LinuxCNC machine.
#
# The document is "hybrid": a standard MTConnect component tree (Controller,
# Path, Axes with Linear/Rotary components and Motion elements) plus a compact
# LinuxCNC extension block (<x:Kinematics>) carrying the kins module name, the
# coordinates string and the joint<->axis map -- the primary contract for a
# FreeCAD auto-configuration plugin.
#
# The DataItems themselves come from the shared registry (observations.py) so
# the probe and the /current and /sample streams cannot drift apart.
#
# Run standalone to dump a probe document from an INI file:
#     python3 -m mtc.device_model path/to/machine.ini

import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from .ini_reader import IniReader
from .observations import build_dataitems, EXT_NS
from . import kinematics as kin

MTC_NS = "urn:mtconnect.org:MTConnectDevices:1.7"
XSI_NS = "http://www.w3.org/2001/XMLSchema-instance"
SCHEMA_VERSION = "1.7"

_LINEAR_UNITS = {
    "mm": "MILLIMETER", "metric": "MILLIMETER", "millimeter": "MILLIMETER",
    "inch": "INCH", "imperial": "INCH", "in": "INCH", "cm": "CENTIMETER",
}
_ANGULAR_UNITS = {
    "degree": "DEGREE", "degrees": "DEGREE", "deg": "DEGREE",
    "radian": "RADIAN", "grad": "DEGREE",
}

@dataclass
class DeviceConfig:
    name: str = "linuxcnc"
    uuid: str = "linuxcnc-0001"
    manufacturer: str = "LinuxCNC"
    linear_units: str = "MILLIMETER"
    angular_units: str = "DEGREE"
    instance_id: str = "1"

    @classmethod
    def from_ini(cls, ini):
        lin = (ini.find("TRAJ", "LINEAR_UNITS", "mm") or "mm").strip().lower()
        ang = (ini.find("TRAJ", "ANGULAR_UNITS", "degree") or "degree").strip().lower()
        return cls(
            name=ini.find("MTCONNECT", "DEVICE_NAME",
                          ini.find("EMC", "MACHINE", "linuxcnc")) or "linuxcnc",
            uuid=ini.find("MTCONNECT", "UUID", "linuxcnc-0001") or "linuxcnc-0001",
            linear_units=_LINEAR_UNITS.get(lin, "MILLIMETER"),
            angular_units=_ANGULAR_UNITS.get(ang, "DEGREE"),
        )


# Namespaces are declared as literal xmlns attributes on the root (below) and
# tags are written with plain / prefixed names.  This avoids ElementTree's
# global register_namespace() state, which mis-handles several default
# namespaces in one process.  Serialized output round-trips through any
# namespace-aware parser exactly as if {uri}Tag had been used.
def _t(tag):
    return tag


def _x(tag):
    return "x:" + tag


def _fmt_vec(vec):
    return " ".join(("%g" % (c if c != 0 else 0.0)) for c in vec)  # avoid -0


def build_device_element(model, config, models=None):
    """Build the <Device> element (standard tree + kinematics extension)."""
    dev = ET.Element(_t("Device"),
                     {"id": "dev_%s" % config.name, "name": config.name,
                      "uuid": config.uuid})
    ET.SubElement(dev, _t("Description"), {"manufacturer": config.manufacturer})

    # Device-level Configuration: coordinate systems + the static frame model.
    if models and models.enabled():
        cfg = ET.SubElement(dev, _t("Configuration"))
        _build_coordinate_systems(cfg)
        if models.base:
            _solid_model(cfg, "dev_base_model", models.base, models)

    # containers maps a component id to its (created, empty) <DataItems> element;
    # the registry loop below fills them so probe and streams stay in lockstep.
    containers = {}
    containers["dev_%s" % config.name] = ET.SubElement(dev, _t("DataItems"))

    components = ET.SubElement(dev, _t("Components"))
    _build_controller(components, containers)
    _build_axes(components, model, config, containers, models)

    for di in build_dataitems(model, config):
        parent = containers.get(di.comp_id)
        if parent is not None:
            _emit_dataitem(parent, di)

    _build_kinematics_extension(dev, model, config)
    return dev


def _emit_dataitem(parent, di):
    type_ = ("x:" + di.type) if di.ext else di.type
    attrs = {"category": di.category, "type": type_, "id": di.id}
    if di.subType:
        attrs["subType"] = di.subType
    if di.units:
        attrs["units"] = di.units
    if di.representation:
        attrs["representation"] = di.representation
    ET.SubElement(parent, _t("DataItem"), attrs)


def _build_controller(parent, containers):
    ctrl = ET.SubElement(parent, _t("Controller"), {"id": "ctrl", "name": "controller"})
    containers["ctrl"] = ET.SubElement(ctrl, _t("DataItems"))
    paths = ET.SubElement(ctrl, _t("Components"))
    path = ET.SubElement(paths, _t("Path"), {"id": "path", "name": "path"})
    containers["path"] = ET.SubElement(path, _t("DataItems"))


def _build_axes(parent, model, config, containers, models=None):
    axes = ET.SubElement(parent, _t("Axes"), {"id": "axes", "name": "axes"})
    comps = ET.SubElement(axes, _t("Components"))
    for axis in model.axes:
        _build_motion_axis(comps, axis, containers, models)
    _build_spindle(comps, containers, models)


def _build_motion_axis(parent, axis, containers, models=None):
    is_linear = axis.kind == "LINEAR"
    aid = axis.letter.lower()
    comp = ET.SubElement(parent, _t("Linear" if is_linear else "Rotary"),
                         {"id": "axis_%s" % aid, "name": axis.letter})
    cfg = ET.SubElement(comp, _t("Configuration"))
    motion = ET.SubElement(cfg, _t("Motion"), {
        "id": "motion_%s" % aid,
        "type": "PRISMATIC" if is_linear else "REVOLUTE",
        "actuation": "DIRECT",
        "coordinateSystemIdRef": "machine",
    })
    # Chain this link to its parent link's motion so the twin nests transforms.
    if models and models.enabled():
        parent_id = models.parent_of("axis_%s" % aid)
        if parent_id and parent_id.startswith("axis_"):
            motion.set("parentIdRef", "motion_%s" % parent_id.split("_", 1)[1])
    vec = axis.vector
    if models and axis.letter in models.invert:
        vec = tuple(-c for c in vec)   # work-carrying axis moves opposite
    ET.SubElement(motion, _t("Axis")).text = _fmt_vec(vec)
    if models and axis.letter in models.axis:
        _solid_model(cfg, "model_%s" % aid, models.axis[axis.letter], models)
    containers["axis_%s" % aid] = ET.SubElement(comp, _t("DataItems"))


def _build_spindle(parent, containers, models=None):
    comp = ET.SubElement(parent, _t("Rotary"), {"id": "spindle", "name": "S"})
    if models and models.spindle:
        cfg = ET.SubElement(comp, _t("Configuration"))
        _solid_model(cfg, "model_spindle", models.spindle, models)
    containers["spindle"] = ET.SubElement(comp, _t("DataItems"))


def _build_coordinate_systems(cfg):
    cs = ET.SubElement(cfg, _t("CoordinateSystems"))
    ET.SubElement(cs, _t("CoordinateSystem"),
                  {"id": "world", "type": "WORLD", "name": "world"})
    machine = ET.SubElement(cs, _t("CoordinateSystem"),
                            {"id": "machine", "type": "MACHINE", "name": "machine",
                             "parentIdRef": "world"})
    ET.SubElement(machine, _t("Origin")).text = "0 0 0"


def _solid_model(cfg, sid, ref, models):
    ET.SubElement(cfg, _t("SolidModel"), {
        "id": sid,
        "href": "/models/%s" % ref.name,
        "mediaType": ref.media,
        "coordinateSystemIdRef": "machine",
        "units": models.units,
        "nativeUnits": models.units,
    })


def _build_kinematics_extension(dev, model, config):
    """Compact LinuxCNC-specific kinematic block for auto-configuration."""
    k = ET.SubElement(dev, _x("Kinematics"), {
        "module": model.kins_module,
        "coordinates": model.coordinates,
        "joints": str(model.joints_count),
    })
    if model.kins_params:
        k.set("params", model.kins_params)
    if model.kinematics_type:
        k.set("type", model.kinematics_type)

    jmap = ET.SubElement(k, _x("JointMap"))
    for joint in model.joints:
        attrs = {"number": str(joint.number), "kind": joint.kind}
        if joint.axis:
            attrs["axis"] = joint.axis
        _set_num(attrs, "min", joint.min_limit)
        _set_num(attrs, "max", joint.max_limit)
        _set_num(attrs, "home", joint.home)
        _set_num(attrs, "homeOffset", joint.home_offset)
        ET.SubElement(jmap, _x("Joint"), attrs)

    for axis in model.axes:
        attrs = {"name": axis.letter, "kind": axis.kind, "vector": _fmt_vec(axis.vector)}
        _set_num(attrs, "min", axis.min_limit)
        _set_num(attrs, "max", axis.max_limit)
        ET.SubElement(k, _x("Axis"), attrs)


def _set_num(attrs, key, value):
    if value is not None:
        attrs[key] = "%g" % value


def build_probe_tree(model, config, creation_time="1970-01-01T00:00:00Z",
                     asset_count=0, models=None):
    """Build the full <MTConnectDevices> ElementTree root."""
    root = ET.Element("MTConnectDevices", {
        "xmlns": MTC_NS,
        "xmlns:xsi": XSI_NS,
        "xmlns:x": EXT_NS,
        "xsi:schemaLocation":
            "urn:mtconnect.org:MTConnectDevices:%s "
            "http://schemas.mtconnect.org/schemas/MTConnectDevices_%s.xsd"
            % (SCHEMA_VERSION, SCHEMA_VERSION),
    })
    ET.SubElement(root, _t("Header"), {
        "creationTime": creation_time,
        "sender": "linuxcnc-mtconnect",
        "instanceId": config.instance_id,
        "version": SCHEMA_VERSION,
        "assetCount": str(asset_count),
        "assetBufferSize": "1024",
        "bufferSize": "131072",
    })
    devices = ET.SubElement(root, _t("Devices"))
    devices.append(build_device_element(model, config, models))
    return root


def probe_xml(model, config, models=None, **kwargs):
    """Return the pretty-printed MTConnectDevices document as a string."""
    root = build_probe_tree(model, config, models=models, **kwargs)
    ET.indent(root, space="  ")
    body = ET.tostring(root, encoding="unicode")
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + body + "\n"


def probe_from_ini(ini_path, **kwargs):
    """Convenience: build a probe document straight from an INI file path."""
    from .models import build_models
    ini = IniReader(ini_path)
    model = kin.build_model(ini)
    config = DeviceConfig.from_ini(ini)
    return probe_xml(model, config, models=build_models(ini, model, config), **kwargs)


def _main(argv):
    if len(argv) != 2:
        sys.stderr.write("usage: python3 -m mtc.device_model MACHINE.ini\n")
        return 2
    sys.stdout.write(probe_from_ini(argv[1]))
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv))
