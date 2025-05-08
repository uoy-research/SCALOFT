# SCALOFT
**Situation Coverage-Based Safety Analysis of an Autonomous Aerial Drone in a Mine Environment**

This repository is publicly available to accompany a paper submission. Link to paper to follow.

# ALOFT Testbed

The SCALOFT repository extends the [ALOFT](https://github.com/uoy-research/ALOFT) framework by introducing **situation coverage-based safety testing** for AAD in a mine environment. 
A single trial script is modified to log drone positions, detect human presence, runtime monitoring of safety violation, and visualize the drone's flight path.
[![Demo Video](thumbnail_image.png)](Drone's Journey(edited).mp4)

[Click here to watch the demo video](Drone's Journey(edited).mp4)




---

## What’s Included

* Modified drone trial script:
  - Check safety violation
  - Logs collisions and mission events
  - Plots drone path and collisions
* Logging of mission events to `drone_mission_log.txt`
* Visualization of the mission path in `drone_journey.png`

---
---
## How to Set Up and Run the Trial

### 1. Setup the ALOFT VM

Follow the instructions in the original [ALOFT repository](https://github.com/uoy-research/ALOFT) to set up the VM environment, Gazebo, and ROS.

Make sure the following are running and available:
- `darknet_ros` for person detection
- `/gazebo/set_model_state` and `/gazebo_bumper` services in Gazebo

---

### 2. Run the Trial

`rosrun your_package_name drone_trial.py <person_present>`

Replace `<person_present>` with:

- `1` → to simulate a **person present** in the environment  
- `0` → to simulate **no person** in the environment

**Example:**

`rosrun drone_controller drone_trial.py 1`


---
## Outputs

* `drone_mission_log.json`: Logs events such as drone positions, person detection, and collisions
* `drone_journey.png`: Visualizes the drone's flight path and any collision points
* Console Output: Displays mission summary, including:
  - Collision status
  - Human presence detection time
  - Total time taken to complete the mission

