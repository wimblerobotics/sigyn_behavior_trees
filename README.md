# sigyn_behavior_trees

ROS 2 (Jazzy) package providing custom BehaviorTree.CPP v4 nodes and their
matching action servers for the Sigyn robot.

## Package contents

### BT nodes

| Class | XML tag | Type | Ports |
|---|---|---|---|
| `SS` | `<SS>` | `SyncActionNode` (no ROS) | `message` (string, required) |
| `SaySomething` | `<SaySomething>` | `BtActionNode` (async, calls action server) | `message` (string, required), `pose` (PoseStamped, optional) |
| `MoveAShortDistanceAhead` | `<MoveAShortDistanceAhead>` | `BtActionNode` (async, calls action server) | `distance` (double, default 0.0 m) |

**`SS`** is a lightweight synchronous node — it prints the message to stdout and
returns `SUCCESS` immediately.  No action server is needed.

**`SaySomething`** sends the message (and an optional pose) to the
`say_something` action server, which logs it and returns success.

**`MoveAShortDistanceAhead`** sends a distance goal to the
`move_a_short_distance_ahead` action server, which drives the robot forward the
requested distance.

### Action servers

| Executable | Action topic | Action type |
|---|---|---|
| `SaySomethingActionServer` | `say_something` | `sigyn_behavior_trees/action/SaySomething` |
| `MoveAShortDistanceAheadActionServer` | `move_a_short_distance_ahead` | `sigyn_behavior_trees/action/MoveAShortDistanceAhead` |

Both servers are also registered as **rclcpp components** (plugins), so they
can be loaded into a component container at runtime.

---

## Building

```bash
cd ~/sigyn_behavior_trees_ws
colcon build --packages-select sigyn_behavior_trees
source install/setup.bash
```

---

## Running

### bt_test1 — interactive SaySomething tree runner

Loads a BT XML file, spins the tree once per 100 ms, and stops when you press
Enter.  The `SaySomethingActionServer` must be running alongside it (the
`bt_test1.launch.py` starts both).

```bash
ros2 launch sigyn_behavior_trees bt_test1.launch.py
```

The default XML is `config/bt_test1.xml`.  Pass a different file via
`xml_path`:

```bash
ros2 launch sigyn_behavior_trees bt_test1.launch.py \
  xml_path:=<path-to-your-tree.xml>
```

### bt_test2 — one-shot SS tree runner

Loads a BT XML file from the command line, ticks it to completion (using
`tickWhileRunning`), then exits.  No action server is needed because `SS` is
a synchronous node.

```bash
ros2 run sigyn_behavior_trees bt_test2 \
  $(ros2 pkg prefix sigyn_behavior_trees)/share/sigyn_behavior_trees/config/bt_say_something_test.xml
```

Or via the launch file (uses `config/bt_test2.xml`):

```bash
ros2 launch sigyn_behavior_trees bt_test2.launch.py
```

### Standalone action servers

```bash
ros2 run sigyn_behavior_trees SaySomethingActionServer
ros2 run sigyn_behavior_trees MoveAShortDistanceAheadActionServer
```

---

## Testing

### Build and run all tests

```bash
cd ~/sigyn_behavior_trees_ws
colcon build --packages-select sigyn_behavior_trees
colcon test  --packages-select sigyn_behavior_trees
colcon test-result --verbose
```

`colcon test` runs:

| Test target | Kind | Description |
|---|---|---|
| `test_ss_node` | **Unit** | Directly instantiates the `SS` node and verifies `SUCCESS` with a valid message and `RuntimeError` when the message port is missing.  No ROS context needed. |
| `test_say_something_node` | **Integration** | Spins a `MockSaySomethingServer` in a background thread and ticks a BT tree containing `SaySomething`.  Covers message-only and message-with-pose goals. |
| `test_move_a_short_distance_ahead_node` | **Integration** | Spins a `MockMoveAShortDistanceAheadServer` and verifies the tree reaches `SUCCESS` and the mock server receives the exact `distance` value from the tree. |
| `cpplint` / `flake8` / `xmllint` | **Lint** | ament style checks. `uncrustify` is excluded (package uses clang-format / Google style). `ament_copyright` is excluded because the Jazzy-shipped version does not recognise `SPDX-License-Identifier` headers. |

### Run only the gtest binaries directly (faster feedback loop)

After building, the test binaries live under
`build/sigyn_behavior_trees/`:

```bash
# Unit test (no ROS needed)
./build/sigyn_behavior_trees/test_ss_node

# Integration tests (need a running ROS context — the binary calls
# rclcpp::init / rclcpp::shutdown internally)
./build/sigyn_behavior_trees/test_say_something_node
./build/sigyn_behavior_trees/test_move_a_short_distance_ahead_node
```

Run a single test case:

```bash
./build/sigyn_behavior_trees/test_say_something_node \
  --gtest_filter=SaySomethingNodeTest.TreeSucceedsWithMessageOnly
```

### Re-run only failed tests

```bash
colcon test --packages-select sigyn_behavior_trees \
  --rerun-failed --output-on-failure
```

---

## nav2 integration

To load the BT nodes into a nav2 BT navigator, add the plugin libraries to
your YAML configuration:

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - sigyn_say_something_plugin_node
      - sigyn_move_a_short_distance_ahead_plugin_node
```

Then reference the nodes by their XML tag names in your BT XML:

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <SaySomething message="Approaching goal" pose="{goal_pose}"/>
      <MoveAShortDistanceAhead distance="0.3"/>
      <SaySomething message="Done"/>
    </Sequence>
  </BehaviorTree>
</root>
```

---

## Adding a new BT node

1. Add an action definition in `action/MyAction.action` and register it in
   `CMakeLists.txt` (`rosidl_generate_interfaces`).
2. Create `include/sigyn_behavior_trees/my_action.hpp` and
   `src/my_action.cpp` following the `SaySomething` pattern.
3. Create an action server in `src/my_action_action_server.cpp` following the
   `SaySomethingActionServer` pattern.
4. Add build targets in `CMakeLists.txt` (plugin library + action server
   library + component registration).
5. Add an integration test in `test/test_my_action_node.cpp` with a mock
   server, following the `test_say_something_node.cpp` pattern.

---

## License

Apache-2.0 — see [LICENSE](LICENSE).
