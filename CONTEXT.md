# MAV1 Nav2 Integration

This context contains the ROS 2 integration boundary between the PX4 MAV1
simulation, Nav2 planning, and the shared arena representation.

## Visualization

**Integrated RViz**:
The manually started MAV1 visualization that overlays the Static Occupancy Map,
arena markers, MAV1 TF, and the MAV1 global plan in the shared `map` frame.
_Avoid_: arena RViz, Nav2 RViz

**Arena Marker Publisher**:
The arena package's `graph_markers.py` node, which exposes the SDF arena and
GeoJSON topology as RViz marker topics and may display an independently
converted PX4 vehicle marker.
_Avoid_: Nav2 map publisher, planner visualizer

**Coordinate Cross-Check**:
The intentional visual comparison of the Arena Marker Publisher's PX4-derived
vehicle marker against Nav2's MAV1 TF and plan, used to expose NED-to-ENU or
origin-alignment disagreement.
_Avoid_: localization source, vehicle control
