# SCALOFT
Situation Coverage-Based Safety Analysis of an Autonomous Aerial Drone in a Mine Environment

# ALOFT Testbed

This repository extends the [ALOFT](https://github.com/uoy-research/ALOFT) framework by introducing **situation coverage-based safety testing** for AAD in a mine environment. 
A single trial script is modified to log drone positions, detect human presence, runtime monitoring of safety violation, and visualize the drone's flight path.

---

## What’s Included

*Modified drone trial script:
  - Check safety violation
  - Logs collisions and mission events
  - Plots drone path and collisions
*Logging of mission events to `drone_mission_log.txt`
*Visualization of the mission path in `drone_journey.png`

---

## 📁 Files

```bash
SCALOFT/
├── drone_trial.py   # Modified ROS script for the trial
├── drone_mission_log.txt                    # Generated log file after trial
├── drone_journey.png                        # Path visualization after trial
├── README.md                                # This file


##  How to Set Up and Run the Trial

### 1. Setup the ALOFT VM

Follow the instructions in the original [ALOFT repository](https://github.com/uoy-research/ALOFT) to set up the VM environment, Gazebo, and ROS.

Make sure the following are running and available:
- `darknet_ros` for person detection
- `/gazebo/set_model_state` and `/gazebo_bumper` services in Gazebo

---

### 2. Run the Trial

```bash
rosrun your_package_name drone_trial.py <person_present>
Replace `<person_present>` with:

- `1` → to simulate a **person present** in the environment  
- `0` → to simulate **no person** in the environment

**Example:**

```bash
rosrun drone controller drone_trial.py 1

