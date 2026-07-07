# Day 2 — Sim-to-Real Guide

**Phases covered:** Phase 2 — Autonomous Navigation · Phase 3 — Human-in-the-loop Delivery

> This guide is for running on the **real myAGV 2023 robot**.
> For simulation-only instructions see `LAB_GUIDE_DAY2.md`.

---

## Before you start

- The robot should be on the floor, roughly where it was when the Day 1 map was saved.
- You need the map files from Day 1: `lab_map.pgm` and `lab_map.yaml`.
- Every step below that says "new terminal" means: SSH in again, start the container, and source (steps 1a–1c).

---

## Common setup (do once per session)

### Step 1 — Connect and enter the container

**1a. SSH from your laptop:**
```bash
ssh ubuntu@<robot-IP>
```

**1b. Start the container** (skip if already running):
```bash
sudo docker start myagv_slam
```

**1c. Enter and source** (repeat in every new terminal you open):
```bash
sudo docker exec -it myagv_slam bash
source /opt/ros/galactic/setup.bash
source /ws/install/setup.bash
```

### Step 2 — Put the Day 1 map in place

```bash
cp /ws/lab_map.pgm  /ws/src/myAGV_lab_project/maps/lab_map.pgm
cp /ws/lab_map.yaml /ws/src/myAGV_lab_project/maps/lab_map.yaml
```

Confirm the yaml points to the right image:
```bash
cat /ws/src/myAGV_lab_project/maps/lab_map.yaml
```
The `image:` line should read `lab_map.pgm`.

### Step 3 — Start the robot base (Window 1)

```bash
ros2 launch myagv_odometry myagv_active.launch.py
```
Confirm it is **not** printing `Serial port verification failed`.

### Step 4 — Start Nav2 (Window 2)

New terminal → enter container → source, then:
```bash
ros2 launch myagv_lab nav2_launch.py
```

### Step 5 — Wait for map server and localisation to come up

`nav2_launch.py` passes `autostart:=true` and bakes the map path directly
into `map_server`'s `yaml_filename` param, so `map_server` and `amcl`
configure and activate themselves — no manual lifecycle commands needed.

Just wait until Window 2 shows:
```
Managed nodes are active
```

If it never reaches that state, check the lifecycle manually:
```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```
Both should read `active`. If either is stuck in `unconfigured`, activate it
by hand as a fallback:
```bash
ros2 param set /map_server yaml_filename /ws/src/myAGV_lab_project/maps/lab_map.yaml
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
```

### Step 6 — Open RViz2 on your laptop

RViz2 runs on the robot but displays on your laptop via X11 forwarding.

From your **laptop** (separate terminal):
```bash
ssh -X ubuntu@<robot-IP>
xhost +
sudo docker exec -it -e DISPLAY=$DISPLAY myagv_slam bash
source /opt/ros/galactic/setup.bash
source /ws/install/setup.bash
rviz2
```

**If RViz2 gives an X11 authentication error**, run on the robot host (outside container):
```bash
xauth extract /tmp/.myxauth $DISPLAY
sudo docker cp /tmp/.myxauth myagv_slam:/tmp/.myxauth
```
Then inside the container:
```bash
export XAUTHORITY=/tmp/.myxauth
rviz2
```

In RViz2:
- **Fixed Frame** → `map`
- Add: `/map` (Map), `/scan` (LaserScan), TF
- Turn **off** RobotModel and Grid to reduce lag

### Step 7 — Set the initial pose

AMCL does not know where the robot is until you tell it.

**In RViz2:**
1. Click **"2D Pose Estimate"** in the toolbar.
2. Click the robot's real location on the map.
3. Drag in the direction the robot faces, then release.

The laser scan should align with the map walls. If it is badly off, redo the estimate.

Verify localisation has converged:
```bash
ros2 run tf2_ros tf2_echo map odom
```
Values should be stable (not jumping by metres).

---

## Phase 2 — Autonomous Navigation

After completing Steps 1–7 above, you can send navigation goals.

### Send a goal from RViz2

1. Click **"2D Nav Goal"** in the toolbar.
2. Click a free (white) area on the map.
3. Drag to set the arrival heading, then release.

The robot plans a path and drives to the goal autonomously.

### Send a goal from the command line

```bash
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### Named waypoints (optional)

To use the named waypoints defined in the code:

1. In RViz2, use **"Publish Point"** to hover over key locations. The `x, y`
   coordinates appear at the bottom of the window — record them.
2. Update `REAL_WAYPOINTS` in `myagv_lab/phase2_nav/nav_node.py` with the real
   values (leave `SIM_WAYPOINTS` untouched — the sim tests depend on it).
3. Rebuild and run:
```bash
cd /ws && colcon build --packages-select myagv_lab && source install/setup.bash
export MYAGV_USE_SIM=0
python3 -m myagv_lab.phase2_nav.nav_node --real
```

---

## Phase 3 — Human-in-the-loop Delivery

Phase 3 uses the full **LLM → PDDL → pyperplan → execution** pipeline on the real
robot. The only difference from Phase 4 is that the `load-package` and
`deliver-package` steps pause and ask **you** to place or remove the package manually
instead of commanding a cobot arm.

> Complete Steps 1–7 (common setup) before running Phase 3.

### Prepare the lab

- Place **`package_A`** at the physical loading area.
- Confirm `REAL_WAYPOINTS` in `nav_node.py` match the real lab layout
  (use the "Publish Point" method from the Phase 2 "Named waypoints" section to
  read coordinates if needed).

### Run with fallback PDDL (no API key needed)

New terminal → enter container → source, then:

```bash
export MYAGV_USE_SIM=0
python3 -m myagv_lab.phase3_human_cobot.human_cobot_manager --real --fallback
```

Or via the ROS2 launch file:
```bash
ros2 launch myagv_lab human_cobot_launch.py use_fallback:=true
```

### Run with LLM translation

Requires `STUDENT_ID` and API keys exported in the container environment:
```bash
export STUDENT_ID="yourlastname"
export DEEPSEEK_API_KEY="sk-..."
export LANGFUSE_PUBLIC_KEY="pk-lf-..."
export LANGFUSE_SECRET_KEY="sk-lf-..."
export LANGFUSE_BASE_URL="https://cloud.langfuse.com"

python3 -m myagv_lab.phase3_human_cobot.human_cobot_manager --real \
  --task "Deliver package_A to the delivery area and return home."
```

Or via launch file:
```bash
ros2 launch myagv_lab human_cobot_launch.py \
  task:="Deliver package_A to the delivery area and return home."
```

### What happens during the run

The manager prints the plan and then executes it step by step.

For every **`navigate`** step:
- The AGV drives autonomously using Nav2.
- You will see progress in Window 2 (Nav2 logs) and the robot moving.

For every **`load-package`** or **`deliver-package`** step:
- The AGV stops at the location.
- The terminal prints:

```
★  HUMAN ACTION REQUIRED
   Please place  package_A  onto the AGV platform at the loading area.
   Press Enter when done…
```

- Physically load or unload the package, then press **Enter**.
- The pipeline resumes immediately.

At the end, the mission timeline is printed showing every status event and timestamp.

### Available scenarios

```bash
# Single package (default)
python3 -m myagv_lab.phase3_human_cobot.human_cobot_manager \
    --real --fallback --scenario deliver_A

# Two packages
python3 -m myagv_lab.phase3_human_cobot.human_cobot_manager \
    --real --fallback --scenario deliver_AB

# Recharge then deliver
python3 -m myagv_lab.phase3_human_cobot.human_cobot_manager \
    --real --fallback --scenario recharge_then_deliver
```

---

## Notes / common issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| "map does not exist" / transform timeout | AMCL not active, or no initial pose set | Complete Step 5 and Step 7 |
| `map_server` "Transitioning failed" | Map path was not set before `configure` (shouldn't happen — `nav2_launch.py` bakes it in — but can occur if you're launching `bringup_launch.py` directly) | Run `ros2 param set /map_server yaml_filename ...` then `configure`/`activate` by hand |
| Localisation jumps by metres | Wrong initial pose | Re-set the 2D Pose Estimate in RViz2 |
| "Robot is out of bounds of the costmap" | Initial pose is outside the map | Re-set to the robot's real position |
| Navigation goal fails / robot aborts | Goal is inside an obstacle or too close to a wall | Pick a clearer spot |
| RViz2 X11 authentication error | Container missing display authority | Run `xauth extract` + `docker cp` from Step 6 |
| AGV moves but ignores `load-package` prompt | Terminal focus is on another window | Click the terminal running the manager and press Enter |
| `Serial port verification failed` (Step 3) | USB cable to robot not connected or wrong port | Check cable and re-run the base launch |
| `Missing environment variables` | API keys not exported in the container shell | Export them manually or add to `~/.bashrc` inside the container |
