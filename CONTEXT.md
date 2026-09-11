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

## Navigation Boundaries

**Planning Result**:
The vehicle-namespaced `nav_msgs/Path` emitted by Nav2's PlannerServer for
visualization and debugging; observing it never authorizes motion.
_Avoid_: flight command, execution command

**Execution Command**:
An explicit Nav2 action request that authorizes a controller to track a
specified path for one vehicle.
_Avoid_: planning result, RViz goal

## Formation

**Leader**:
The sole vehicle that owns the final mission goal and runs the Nav2 navigation
stack for a formation.
_Avoid_: primary planner, fleet controller

**Follower**:
A vehicle that maintains a specified formation displacement from the Leader
without independently owning the formation's final mission goal.
_Avoid_: secondary leader, independent mission vehicle

**Formation Navigation Mission**:
A multi-vehicle mission in which the Leader navigates to the final goal while
Followers maintain their formation slots and collectively reach mission completion.
_Avoid_: independent three-vehicle navigation

**Formation Slot**:
A follower's displacement defined in the Leader's body FLU frame and converted
to a target in the shared ENU `map` frame using the Leader's current yaw.
_Avoid_: fixed world offset, follower mission goal

**Fixed V-Slot**:
The first Formation Slot arrangement: MAV2 is 0.8 m behind and 0.8 m left of
the Leader, while MAV3 is 0.8 m behind and 0.8 m right, with no vertical offset.
_Avoid_: fixed ENU offset, dynamic formation mode

**Mission Goal**:
A final `map`-frame destination for a Formation Navigation Mission, selected
either directly as a pose or by resolving an arena topology-node identifier.
_Avoid_: Planning Result, follower slot

**Topology Goal Resolution**:
The lookup of an arena GeoJSON node identifier to its `map`-frame coordinate;
it selects a Mission Goal but does not prescribe the route to reach it.
_Avoid_: route execution, direct PX4 command

**Flight Level**:
The common ENU altitude maintained by all vehicles during a Formation Navigation
Mission; its initial default is 3.0 m and is mission-configurable.
_Avoid_: Nav2 path z, per-follower altitude

**Abort Hold**:
The formation safety state that cancels navigation and commands controllable
vehicles to maintain their current flight-level positions pending Landing or recovery.
_Avoid_: immediate land, continued navigation
