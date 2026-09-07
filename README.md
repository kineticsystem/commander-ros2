# Commander

A single ROS 2 action server that commands the [StepIt](https://github.com/kineticsystem/stepit)
robot by executing *objectives*, written as [BehaviorTree.CPP](https://www.behaviortree.dev)
trees.

A client never talks to a controller directly. It sends the **name** of an
objective and a **payload** holding its parameters to one action:

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: OffsetJointsBy,
    payload: '{joints: [joint1, joint3], offset: -6.28, duration: 4.0}'}"
```

The action server itself comes from [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2)
(`BT::TreeExecutionServer`): it loads every objective and every behavior found in
the folders listed in its parameters, so a new objective is added by dropping an
XML file into a package, without touching the server.

## Table of Contents <!-- omit in toc -->

- [Packages](#packages)
- [The command](#the-command)
- [The OffsetJointsBy objective](#the-offsetjointsby-objective)
- [The MoveJointsTo objective](#the-movejointsto-objective)
- [The ActivateController objective](#the-activatecontroller-objective)
- [Build and run](#build-and-run)
- [Tests](#tests)
- [Adding a new objective](#adding-a-new-objective)

## Packages

Each package has one concern, and one only.

| Package | Role |
|---|---|
| `commander_objectives` | The objectives and the subtrees they are built from: BehaviorTree XML files, no code. |
| `commander_behaviors` | The behaviors the objectives are built from. The only place that knows the topics, actions and services of the robot. |
| `commander_server` | The single action server, its parameters and its launch file. It knows nothing about the robot. |
| `commander_tests` | Tests: the logic of the behaviors, the payload of a command, and the objectives run end to end against a fake robot. |

`BehaviorTree.ROS2` is not released as a Debian package, so it is checked out as
a git submodule under [`modules`](modules), next to `src`. Colcon builds every
package it finds under the workspace root, so its packages are built together
with ours.

## The command

The goal of the action is `btcpp_ros2_interfaces/action/ExecuteTree`:

| Field | Meaning |
|---|---|
| `target_tree` | The name of the objective, i.e. the `ID` of a `<BehaviorTree>`. |
| `payload` | Its parameters, as a YAML (and therefore also JSON) map. |

The server parses the payload and writes every parameter into the **global
blackboard** of the tree, where the behaviors read it through the `@` prefix,
e.g. `{@offset}`. A payload that is not a map of scalars and lists is refused,
and the goal is rejected before the tree is created.

Values are typed as follows, so that the ports of the behaviors read them
without any further conversion:

| Payload | Blackboard |
|---|---|
| `offset: -6.28` | `double` |
| `duration: 3` | `double` |
| `controllers: velocity_controller` | `std::string` |
| `controllers: '5'` (quoted) | `std::string` |
| `joints: [joint1, joint2]` | `std::vector<std::string>` |
| `positions: [0.0, 1.5]` | `std::vector<double>` |

## The OffsetJointsBy objective

[`offset_joints_by.xml`](src/commander_objectives/objectives/offset_joints_by.xml)
offsets one or more joints, at the same time, **relative** to the position they
have when the objective starts. It is the relative counterpart of
`MoveJointsTo`: the two names say how they differ, *by* an amount against *to* a
position.

| Parameter | Required | Meaning |
|---|---|---|
| `joints` | yes | The joints to move, e.g. `[joint1, joint3]`. |
| `offset` | yes | The signed displacement of each joint, in radians. |
| `duration` | no | Time to complete the motion, in seconds. Defaults to 5. |

The tree reads the current position of the joints from `/joint_states`, turns
the offset into absolute joint targets, and sends them as a single waypoint to
the `FollowJointTrajectory` action of the `joint_trajectory_controller`:

```
Sequence
├── SubTree EnsureControllers  (activates joint_trajectory_controller)
├── GetJointPositions          (reads /joint_states)             -> current_positions
├── OffsetJointPositions       (pure logic: no ROS)              -> target_positions
└── FollowJointTrajectory      (calls the trajectory controller)
```

The objective starts by making sure the trajectory controller is the one
driving the robot: it cannot send a trajectory otherwise. That first step is the
`EnsureControllers` subtree, shared with `ActivateController`.

**Sign convention.** The offset is signed, and its sign is the one of the joint
positions themselves: a **negative** offset decreases the joint position, which
on the StepIt motors means turning **clockwise**, as in the StepIt README, where
`joint1` is rotated 6.28 rad clockwise by commanding the position `-6.28`. There
is no separate direction parameter: `offset: -6.28` is one turn clockwise,
`offset: 1.57` a quarter turn counterclockwise.

## The MoveJointsTo objective

[`move_joints_to.xml`](src/commander_objectives/objectives/move_joints_to.xml) is
the absolute counterpart of `OffsetJointsBy`: it moves the joints **to** the given
positions, whatever position they are in when the objective starts.

| Parameter | Required | Meaning |
|---|---|---|
| `joints` | yes | The joints to move, e.g. `[joint1, joint2]`. |
| `positions` | yes | The absolute target of each joint, in radians. One per joint. |
| `duration` | no | Time to complete the motion, in seconds. Defaults to 5. |

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: MoveJointsTo,
    payload: '{joints: [joint1, joint2], positions: [0.0, 1.57], duration: 3.0}'}"
```

```
Sequence
├── SubTree EnsureControllers  (activates joint_trajectory_controller)
└── FollowJointTrajectory      (calls the trajectory controller)
```

It needs no C++ of its own. The positions are already the targets, so neither
the current state of the robot nor an offset to apply to it come into
it: `GetJointPositions` and `OffsetJointPositions` are simply not in the tree,
and the payload goes straight to the controller. Running it twice leaves the
robot where it was the first time.

## The ActivateController objective

[`activate_controller.xml`](src/commander_objectives/objectives/activate_controller.xml)
stops whichever controller is currently driving the robot and activates the
requested one instead.

| Parameter | Required | Meaning |
|---|---|---|
| `controllers` | yes | The controllers to activate, e.g. `[velocity_controller]`. A single name may be written as a scalar. |

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: ActivateController, payload: '{controllers: velocity_controller}'}"
```

```
ActivateController                        (the objective: reads the payload)
└── SubTree EnsureControllers             (the reusable part)
    ├── GetActiveControllers              (calls /controller_manager/list_controllers)
    └── SwitchController                  (calls /controller_manager/switch_controller)
```

The objective itself is only an adapter: it forwards `{@controllers}` from the
payload into the `EnsureControllers` subtree, which holds the actual work. Any
objective that needs a given controller calls the same subtree with a fixed
name, as `OffsetJointsBy` does:

```xml
<SubTree ID="EnsureControllers" controllers="joint_trajectory_controller"/>
```

The tree first asks the controller manager which controllers are running, and
stops only those that **own a command interface**: a broadcaster such as the
`joint_state_broadcaster` reads the state of the robot without driving it, and
must keep running, or every other objective would go blind.

Two details are worth knowing:

- The switch is *not* delegated to the `FORCE_AUTO` strictness of the controller
  manager. On StepIt each joint exports both a `position` and a `velocity`
  command interface, so `joint_trajectory_controller` and `velocity_controller`
  do not conflict: asking the controller manager to resolve the switch by itself
  leaves **both** of them active. Measured on the robot, hence the explicit list
  and stop.
- The strictness used is `best_effort`, so activating the controller that is
  already running is not an error. An unknown controller still is, and because
  the controller manager applies a switch atomically, the running controller
  survives a command that could not be honoured.

## Build and run

Everything is built and run inside a Docker container. From the root of the
repo, create the image and the container (see [docker/README.md](docker/README.md)):

```bash
./docker/dock.sh commander-ros2 build
./docker/dock.sh commander-ros2 start
```

Check out the repository including its submodules:

```bash
git clone --recurse-submodules <this repo>
```

If the `--recurse-submodules` switch was missed, the submodules can be cloned
afterwards with:

```bash
git submodule update --init --recursive
```

Inside the container, install the dependencies and build:

```bash
./bin/update.sh    # rosdep install
./bin/build.sh     # colcon build
```

Start the StepIt robot in its own container, as described in its README, then
start the commander:

```bash
source install/setup.bash
ros2 launch commander_server commander.launch.py
```

From another shell in the container, rotate `joint1` and `joint3` by one turn
clockwise:

```bash
source install/setup.bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: OffsetJointsBy,
    payload: '{joints: [joint1, joint3], offset: -6.28, duration: 4.0}'}"
```

The running tree can be inspected with [Groot2](https://www.behaviortree.dev/groot),
which connects to port 1667.

## Tests

```bash
./bin/test.sh
```

or, for this project alone:

```bash
colcon test --packages-select commander_tests --event-handlers console_direct+
```

`test_offset_joints_by_objective` runs the real objective XML and the real
behaviors against a fake robot that publishes `/joint_states` and serves
`FollowJointTrajectory`, so no hardware and no controller are needed.

## Adding a new objective

1. Write the XML in `src/commander_objectives/objectives`. Nothing else to do:
   the folder is already loaded by the server. A step that more than one
   objective needs belongs in `src/commander_objectives/subtrees` instead, and
   is called with `<SubTree ID="..."/>`.
2. If it needs a new behavior, add it to `src/commander_behaviors` and register
   it in `commander_behaviors::registerNodes`. It is picked up automatically,
   because the whole package is loaded as one plugin.
3. Add a test to `src/commander_tests`.

The three objectives shipped here, `OffsetJointsBy`, `MoveJointsTo` and
`ActivateController`, are built from five behaviors and show every shape a
behavior can take: a ROS action client (`FollowJointTrajectory`), service
clients (`GetActiveControllers`, `SwitchController`), a subscriber
(`GetJointPositions`) and pure logic (`OffsetJointPositions`).
