# Mission Coordination Project - ROS

**University of Évry Paris Saclay - M2 SAAS**

This project implements various coordination and navigation strategies for a multi-robot system in a simulated environment using ROS 1 and Gazebo. The goal is to coordinate three autonomous robots to reach their respective flags while avoiding static obstacles (buildings, bus, tower) and preventing dynamic collisions at a central intersection.



---

## 📌 Project Structure

The repository contains the following key components:

* **`evry_project_description/`**: Defines the simulation environment and resources.
* **`worlds/`**: Contains the world files (e.g., `lev1.world`) defining the layout.
* **`models/`**: Custom Gazebo models added for this project (e.g., specific obstacles like the bus and tower structures) to create a robust testing environment.
* **`evry_project_strategy/`**: Contains the Python scripts implementing different navigation strategies.
    * `nodes/agent.py`: Baseline strategy (Timing-based).
    * `nodes/agent_robust.py`: Reactive avoidance using FSM.
    * `nodes/agent_coord.py`: Geometric coordination with bypass maneuvers.
    * `nodes/agent_priority.py`: Decentralized priority-based traffic control.
    * `nodes/agent_apf.py`: Artificial Potential Fields (APF) for smooth navigation.
* **`launch/`**: Launch files to execute the strategies.

---

## 🚀 Strategies Implemented

### 1. Baseline Timing (`agent.py`)
* **Method:** PID control for motion + Static Time Delay for coordination.
* **Logic:** Robots start at staggered intervals (0s, 2s, 4s) to avoid collision at the center.
* **Pros/Cons:** Simple to implement but highly fragile; fails with static obstacles.

### 2. Reactive Avoidance (`agent_robust.py`)
* **Method:** Finite State Machine (GO <-> AVOID).
* **Logic:** Stops and rotates in place when an obstacle is detected by sonar (< 4.0m).
* **Pros/Cons:** Avoids walls but gets stuck in local minima (oscillates) against large obstacles like the bus.

### 3. Geometric Coordination (`agent_coord.py`)
* **Method:** Odometry-based Bypass Maneuver.
* **Logic:** Calculates absolute goal coordinates. Implements a "blind drive" state to physically clear the side of large obstacles before turning back to the goal.
* **Pros/Cons:** Solves local minima; robust against large static obstacles.

### 4. Priority Coordination (`agent_priority.py`)
* **Method:** Decentralized Peer-to-Peer Communication.
* **Logic:** Robots subscribe to each other's positions. A "Critical Zone" (3m radius) is defined at the center. If a higher-priority robot (lower ID) is in the zone, others yield.
* **Pros/Cons:** Robust traffic control; handles dynamic delays effectively.

### 5. Artificial Potential Fields (`agent_apf.py`)
* **Method:** Physics-based Force Summation.
* **Logic:** Robot moves based on the sum of Attractive Forces (to goal) and Repulsive Forces (from obstacles).
* **Pros/Cons:** Smooth, continuous paths; efficient but requires careful tuning of force constants.



---

## 🛠️ Installation & Usage

### Prerequisites
* **ROS Noetic** (or compatible ROS 1 distribution)
* **Gazebo Simulator**
* **Python 3** with `rospy`

### Setup

1.  **Clone this repository** into your catkin workspace `src` folder:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/andreikotkov/Mission_Coordination_project.git
    ```
    
2.  **Build the workspace**:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

3.  **Make Python scripts executable**:
    ```bash
    cd src/Mission_Coordination_project/evry_project_strategy/nodes
    chmod +x *.py
    ```

### Running the Simulation

1.  **Launch the Environment:**
    Open a terminal and run the following command to start the Gazebo simulation with the robots and custom obstacles:
    ```bash
    roslaunch evry_project_description simu_robot.launch
    ```

2.  **Launch the Strategy:**
    In a separate terminal, launch the robust strategy for all 3 robots:
    ```bash
    roslaunch evry_project_strategy agent_robust.launch nbr_robot:=3
    ```
    *(Note: You can modify the launch file to point to other strategies, such as `agent_priority.py` or `agent_coord.py`)*

---

## 👥 Contributors

* Batuhan IRIOL
* Andrei KOTKOV
