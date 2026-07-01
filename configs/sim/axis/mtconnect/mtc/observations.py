# Shared DataItem registry.
#
# One source of truth for the DataItems exposed by the agent so the /probe
# document (device_model) and the /current and /sample streams (streams) cannot
# drift apart.  Each DataItemDef records enough to emit both the <DataItem>
# definition and its streamed observation (element name + component grouping).

from dataclasses import dataclass

from .kinematics import AXIS_LETTERS

# LinuxCNC extension namespace for DataItems with no standard MTConnect type
# (declared as xmlns:x on the probe and stream roots; types use "x:" prefix).
EXT_NS = "urn:linuxcnc:mtconnect:1"


@dataclass
class DataItemDef:
    id: str
    category: str        # SAMPLE / EVENT / CONDITION
    type: str            # MTConnect type, e.g. POSITION, EXECUTION
    comp_id: str         # component id this item lives under
    comp_type: str       # component element, e.g. Linear, Controller, Path
    comp_name: str
    subType: str = None
    units: str = None
    name: str = None
    representation: str = None   # e.g. "TABLE" for WORK_OFFSET
    ext: bool = False            # extension type: emit as "x:<TYPE>" / <x:Element>

    @property
    def element(self):
        """CamelCase observation element name, e.g. POSITION -> Position."""
        return "".join(p.capitalize() for p in self.type.split("_"))


def build_dataitems(model, config):
    """Return the full ordered list of DataItemDefs for a machine model."""
    dev = config.name
    dev_id = "dev_%s" % dev
    items = [
        DataItemDef("avail", "EVENT", "AVAILABILITY", dev_id, "Device", dev),
        DataItemDef("assetchg", "EVENT", "ASSET_CHANGED", dev_id, "Device", dev),
        DataItemDef("assetrm", "EVENT", "ASSET_REMOVED", dev_id, "Device", dev),
        DataItemDef("estop", "EVENT", "EMERGENCY_STOP", "ctrl", "Controller", "controller"),
        DataItemDef("mode", "EVENT", "CONTROLLER_MODE", "ctrl", "Controller", "controller"),
        DataItemDef("execution", "EVENT", "EXECUTION", "path", "Path", "path"),
        DataItemDef("program", "EVENT", "PROGRAM", "path", "Path", "path"),
        DataItemDef("line", "EVENT", "LINE_NUMBER", "path", "Path", "path", subType="ACTUAL"),
        DataItemDef("pathfeed", "SAMPLE", "PATH_FEEDRATE", "path", "Path", "path",
                    units="MILLIMETER/SECOND"),
        DataItemDef("feedovr", "SAMPLE", "PATH_FEEDRATE", "path", "Path", "path",
                    subType="OVERRIDE", units="PERCENT"),
        DataItemDef("toolnum", "EVENT", "TOOL_NUMBER", "path", "Path", "path"),
        DataItemDef("toolasset", "EVENT", "TOOL_ASSET_ID", "path", "Path", "path"),
        # Active work coordinate system (G54..G59.3) + G92, as a TABLE keyed by
        # the offset name with per-axis Cells.  Mirrors g5x_index/g5x_offset.
        DataItemDef("workoffset", "SAMPLE", "WORK_OFFSET", "path", "Path", "path",
                    representation="TABLE"),
        # Applied tool length offset (G43), keyed by active tool; no standard
        # MTConnect live type, so a LinuxCNC extension (x:TOOL_OFFSET).
        DataItemDef("tooloffset", "SAMPLE", "TOOL_OFFSET", "path", "Path", "path",
                    representation="TABLE", ext=True),
        # Active XY coordinate-system rotation (G10 L2 R). Extension type.
        DataItemDef("xyrotation", "SAMPLE", "COORDINATE_ROTATION", "path", "Path",
                    "path", units="DEGREE", ext=True),
    ]

    for axis in model.axes:
        aid = axis.letter.lower()
        cid = "axis_%s" % aid
        if axis.kind == "LINEAR":
            comp_type, dtype, units = "Linear", "POSITION", config.linear_units
        else:
            comp_type, dtype, units = "Rotary", "ANGLE", config.angular_units
        items.append(DataItemDef("pos_%s" % aid, "SAMPLE", dtype, cid, comp_type,
                                 axis.letter, subType="ACTUAL", units=units))
        items.append(DataItemDef("poscmd_%s" % aid, "SAMPLE", dtype, cid, comp_type,
                                 axis.letter, subType="COMMANDED", units=units))

    items += [
        DataItemDef("spdl_speed", "SAMPLE", "ROTARY_VELOCITY", "spindle", "Rotary", "S",
                    subType="ACTUAL", units="REVOLUTION/MINUTE"),
        DataItemDef("spdl_speed_cmd", "SAMPLE", "ROTARY_VELOCITY", "spindle", "Rotary", "S",
                    subType="COMMANDED", units="REVOLUTION/MINUTE"),
        DataItemDef("spdl_mode", "EVENT", "ROTARY_MODE", "spindle", "Rotary", "S"),
        DataItemDef("spdl_dir", "EVENT", "DIRECTION", "spindle", "Rotary", "S", subType="ROTARY"),
    ]
    return items


def axis_index(letter):
    """Index of an axis letter into a 9-element (XYZABCUVW) position tuple."""
    return AXIS_LETTERS.index(letter)
