# 🛰️ Mission Coordination Project (ROS1)

This project explores **multi-robot autonomous navigation and coordination** in an **unknown, obstacle-rich environment** using **ROS1** and **Gazebo**.  
Multiple navigation strategies are implemented and compared, ranging from simple reactive control to advanced Artificial Potential Fields with geometric awareness.

---

## 📦 Environment Setup

### 1) Connect to RDS
Log in to **The Construct RDS** platform:  
https://app.theconstructsim.com/#/

Create a **ROS1 empty project (rosject)**.

---

### 2) Clone the Repository

Open a terminal inside your rosject and execute the following commands **in order**:

```bash
cd ~/catkin_ws/src
git clone https://github.com/andreikotkov/Mission_Coordination_project.git

cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

---

## 📁 Project Structure

```text
Mission_Coordination_project/
├── evry_project_description/
│   └── launch/
│       └── simu_robot.launch
│
├── evry_project_strategy/
│   ├── nodes/
│   └── launch/
```

- **Strategy nodes:**  
  `~/catkin_ws/src/Mission_Coordination_project/evry_project_strategy/nodes`

- **Launch files:**  
  `~/catkin_ws/src/Mission_Coordination_project/evry_project_strategy/launch`

---

## 🚀 Running the Simulation

### Launch Gazebo

```bash
roslaunch evry_project_description simu_robot.launch
```

---

### Launch a Strategy

Available strategies:
- `agent_robust.launch`
- `agent_coord.launch`
- `agent_apf.launch`
- `agent_priority.launch`

Choose the number of robots (**1 to 4**):

```bash
roslaunch evry_project_strategy <strategy_name>.launch nbr_robot:=3
```

---

## ⚙️ System Configuration & Assumptions

-  Three unicycle robots
-  Front distance line sensor (sonar)
-  Real-time distance-to-goal feedback
-  Environment with real-world obstacles (buildings, trees, vehicles)
-  No prior map knowledge (pure online planning)

---

## 🧠 Navigation Strategies

---

## 1) Reactive Distance Minimization (agent_robust.py)

A purely reactive navigation strategy based on distance reduction.

### Description
- The robot does not compute the goal position
- It attempts to reduce the distance to the goal
- If the distance increases, the robot rotates and retries
- When an obstacle is detected, the robot rotates until free space is found

### Characteristics
- Simple and robust
- No geometric reasoning
- Non-optimal paths

### Results

https://github.com/user-attachments/assets/1bec0dd6-e7b1-4378-8e5d-4a4810904105


---

## 2) Goal-Oriented Geometric Navigation (agent_coord.py)

A deterministic strategy based on explicit goal localization.

### Description
- The absolute goal position (x, y) is computed at startup
- The robot continuously aligns toward the goal
- Forward motion follows the shortest geometric path

### Obstacle Avoidance
1. **Avoid Turn** – rotate away from the goal by a minimal safe angle  
2. **Avoid Straight** – move straight for a short distance to bypass the obstacle  
3. Re-align toward the goal and resume navigation

### Characteristics
- Directed and efficient
- Predictable motion
- Limited adaptability in dense environments

### Results

https://github.com/user-attachments/assets/e326e5e9-e19f-45aa-9e56-888363f07a48



---

## 3) Priority-Based Coordination Strategy (agent_priority.py)

A decentralized multi-robot coordination strategy based on peer-to-peer communication and priority rules.

### Method
- Decentralized robot-to-robot communication
- Each robot subscribes to the positions of other robots
- No centralized controller is required

### Coordination Logic
- A **Critical Zone** with a radius of 3 meters is defined at the environment center
- Each robot is assigned a **priority level** based on its ID  
  (lower ID → higher priority)
- If a higher-priority robot enters the Critical Zone:
  - Lower-priority robots **yield**
  - Yielding robots stop or wait until the zone is cleared
- Once the zone is free, waiting robots resume navigation

### Characteristics
- Robust traffic management
- Prevents deadlocks in narrow or shared spaces
- Handles asynchronous motion and dynamic delays well



### Results

https://github.com/user-attachments/assets/b9b54d77-c8b8-4350-9d01-3fc0184b4613


---


## 4) Geometrically-Aware Artificial Potential Field (agent_apf.py)

An advanced navigation method based on Artificial Potential Fields with geometry awareness.

### Description
- The robot is modeled as a square, not a point
- A virtual safety buffer protects robot corners
- Prevents wall clipping during rotations

### Forces
- **Attractive force** pulls the robot toward the goal
- **Repulsive force** pushes the robot away from obstacles

### Characteristics
- Smooth and realistic motion
- Strong obstacle avoidance
- Geometry-safe navigation

### Results

https://github.com/user-attachments/assets/c2d3ad6d-f9b4-408b-b6eb-1509304d5330

### Tactical Retreat and Scan Logic

To handle complex traps (e.g., trees or blind corners), the strategy implements a **Tactical Retreat and Scan** mechanism:
1. **Critical Reverse**  
   If the robot gets dangerously close to an obstacle (< 0.9 m), it immediately stops and reverses in a straight line to regain maneuvering space.

2. **Scan and Search**  
   The robot rotates in place (ignoring the goal direction) until the sensor detects a sufficiently open path (> 3.5 m).

3. **Clearance Enforcement**  
   After dodging, the robot forces a short forward motion to ensure the robot’s body fully clears the obstacle.


https://github.com/user-attachments/assets/d64f9621-806f-4b54-9a1e-184988b72ca8


## 📊 Strategy Comparison

| Strategy | Goal Knowledge | Geometry Awareness | Coordination | Path Quality | Robustness |
|--------|----------------|-------------------|--------------|--------------|------------|
| agent_robust.py | Distance only | No | No | Low | High (simple logic) |
| agent_coord.py | Full (x, y) | No | No | Medium | Medium |
| agent_priority.py | Full (x, y) | No | Yes (P2P) | Medium | High |
| agent_apf.py | Full (x, y) | Yes | No | High | High |


---




## 👤 Authors

**Andrei Kotkov**  
GitHub: https://github.com/andreikotkov

**Batuhan Iriol**

GitHub: https://github.com/batuhaniriol
