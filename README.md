# 16662 CMU - Robot Autonomy Assignments Solutions
### Course: **Robot Autonomy (16-662)**  
**Professor:** Oliver Kroemer  
**TAs:** Vibhakar Mohta, Abhinav Gupta  

This repository contains my solutions to the homework assignments for the **Robot Autonomy** course at Carnegie Mellon University. The assignments focus on core topics such as kinematics, control, collision detection, motion planning, task planning, and reinforcement learning, using the MuJoCo simulation environment and Python. The final project involves using a real Franka robotic arm for the decluttering and shelf-stocking task.

---

## Table of Contents
1. [Setup](#setup)
2. [Homework 1: Kinematics and Control](#homework-1-kinematics-and-control)
    - PID Control
    - Forward Kinematics
    - Inverse Kinematics
3. [Homework 2: Collision Detection and Motion Planning](#homework-2-collision-detection-and-motion-planning)
    - Cuboid-Cuboid Collision Detection
    - RRT and PRM Motion Planning
4. [Homework 3: Symbolic Task Planning](#homework-3-symbolic-task-planning)
5. [Homework 4: Q-Learning](#homework-4-q-learning)
6. [Final Project: Decluttering and Shelf Stocking with a Franka Arm](#final-project-decluttering-and-shelf-stocking-with-a-franka-arm)
7. [Evaluation and Results](#evaluation-and-results)

---

## Setup

Follow these steps to set up the environment and dependencies for running the code in this repository.

1. Clone the repository:
    ```bash
    git clone https://github.com/your_username/robot_autonomy_solutions.git
    cd robot_autonomy_solutions
    ```

2. Create and activate a Python virtual environment:
    ```bash
    python3 -m venv <your_env_name>
    source ./<your_env_name>/bin/activate
    ```

3. Install dependencies:
    ```bash
    python -m pip install mujoco matplotlib numpy quaternion
    ```

4. Ensure that MuJoCo is correctly set up and installed. Follow the [MuJoCo Installation Guide](https://mujoco.org/download) for more details.

---

## Homework 1: Kinematics and Control

This assignment focuses on implementing controllers and kinematic models for a robotic arm, specifically the Franka Emika Panda.

### 1.1 PID Control

- **Goal:** Implement PID controllers (force and impedance) to control the Franka arm's interaction with a whiteboard in the MuJoCo simulation.
- **Tasks:** 
    - Implement the impedance and force controllers.
    - Simulate the interaction of the Franka arm with a static whiteboard and an oscillating whiteboard.
    - Generate CSV logs of the control results.
  
- **Instructions for running the simulation:**
    1. Part 1 (Static Whiteboard):
        ```bash
        python PandaArmControl_Part1.py
        ```
    2. Part 2 (Oscillating Whiteboard):
        ```bash
        python PandaArmControl_Part2.py
        ```

- **Results:**
    - Force vs. Time plots for both controllers (Impedance and Force) in both scenarios.
    - Screenshots and analysis of the differences between the static and oscillating cases.

### 1.2 Forward Kinematics

- **Goal:** Compute the forward kinematics for the Franka arm and generate the Jacobian matrix.
- **Tasks:** 
    - Implement FK in `Franka.py`.
    - Compute end-effector poses for given joint configurations.

- **Results:**
    - End-effector pose for the following joint configurations:
      - `q1 = [0°, 0°, 0°, 0°, 0°, 0°, 0°]`
      - `q2 = [0°, 0°, -45°,-15°,20°,15°,-75°]`
      - `q3 = [0°, 0°, 30°, -60°,-65°,45°,0°]`

### 1.3 Inverse Kinematics

- **Goal:** Implement inverse kinematics using the damped least squares method to compute joint angles for a desired end-effector pose.
- **Tasks:** 
    - Implement the IK method in `Franka.py`.
    - Validate the IK solver by moving the robot to computed joint angles.

- **Results:**
    - Computed joint angles for a goal end-effector pose.
    - Screenshot of the robot reaching the target configuration in the simulation.

---

## Homework 2: Collision Detection and Motion Planning

### 2.1 Cuboid-Cuboid Collision Detection

- **Goal:** Implement a collision detection algorithm using the Separating Axis Theorem.
- **Tasks:** 
    - Implement `CheckBoxBoxCollision()` in `RobotUtil.py`.
    - Test collision detection with cuboids and visualize bounding boxes for the Franka arm.

- **Results:**
    - List of yes/no responses for whether the cuboids in Table 1 are colliding with the reference cuboid.
    - Screenshot of the robot's bounding boxes in the home configuration (all joint angles set to 0).

### 2.2 RRT and PRM Motion Planning

- **Goal:** Implement RRT and PRM algorithms to plan collision-free paths for the Franka arm.
- **Tasks:** 
    - Implement RRT in `RRTQuery.py`.
    - Implement PRM in `PRMGenerator.py` and `PRMQuery.py`.
    - Visualize the generated paths in the MuJoCo simulation.

- **Results:**
    - **RRT**: Video of the robot executing an RRT-based plan (without path shortening).
    - **RRT with Path Shortening**: Video of the robot executing a shortened RRT-based plan.
    - **PRM**: Video of the robot executing a PRM-based plan.

---

## Homework 3: Symbolic Task Planning

### 3.1 Expanding the Set of Actions

- **Goal:** Add new actions to the task planning domain.
- **Tasks:**
    - Add actions for moving between rooms and cutting fruit.
    - Define preconditions and effects for these actions in the planner.

### 3.2 Implementing Discrete Search

- **Goal:** Implement Dijkstra and A* search algorithms to find optimal plans in a symbolic task planning domain.
- **Tasks:** 
    - Implement Dijkstra and A* search.
    - Compare the two algorithms in terms of the number of vertices explored and the length of the final plan.

- **Results:**
    - Number of vertices explored and length of the final plan for both Dijkstra and A*.
    - Screenshot of the final plan.

---

## Homework 4: Q-Learning

### 4.1 Environment and Agent Setup

- **Goal:** Implement a Q-learning agent to solve a gridworld problem, comparing its performance to a random agent.
- **Tasks:**
    - Implement the environment in `environment.py`.
    - Implement the Q-learning algorithm in `agents.py`.
    - Train the Q agent and compare its performance to the random agent.

### 4.2 Training and Evaluation

- **Results:**
    - **Training Plot:** Average rewards during training (`q_learning_training.png`).
    - **Q-values Plot:** Visualization of the learned Q-values (`q_values.png`).
    - Videos showing the performance of the random agent and Q agent after training.

---

## Final Project: Decluttering and Shelf Stocking with a Franka Arm

### 5.1 Project Overview

- **Goal:** Develop a robotic system to perform decluttering and shelf-stocking tasks using a real Franka Emika Panda robotic arm.
- **Tasks:**
    - Implement perception and control algorithms to identify and grasp objects from a cluttered environment.
    - Use motion planning algorithms to accurately place the objects on designated shelves.
    - The system should autonomously perform the task while avoiding collisions.

### 5.2 Implementation Details

- **Perception:** Used a camera setup to detect objects in a cluttered environment. Employed techniques such as 2D to 3D mapping for object localization.
- **Grasping:** Implemented a grasp planner to compute optimal grasp points for different objects.
- **Motion Planning:** Used RRT and PRM-based planning for collision-free navigation to pick and place objects.

### 5.3 Results

- **Videos:** Videos of the Franka arm performing decluttering and shelf-stocking tasks.
    - [Watch Video: Decluttering and Shelf-Stocking Task](final_project/Team8a.mp4)


- **Screenshots:** Include screenshots of the Franka arm during the execution of the tasks.

---

## Evaluation and Results

### Homework 1

- **PID Control:** 
    - Force plots for static and oscillating whiteboard cases.
    - Screenshots of the Franka arm in both cases.

- **Kinematics:**
    - Computed FK and IK results with corresponding screenshots.

### Homework 2

- **Collision Detection:**
    - List of yes/no responses for collision detection tests.
    - Screenshot of Franka arm's bounding boxes.

- **Motion Planning:**
    - Videos of RRT and PRM-based motion plans.

### Homework 3

- **Task Planning:** 
    - Vertices explored and final plan length for Dijkstra and A*.

### Homework 4

- **Q-Learning:** 
    - Training rewards plot.
    - Q-values plot.
    - Videos of random agent and Q agent rollouts.

### Final Project

- **Decluttering and Shelf Stocking:** 
    - Videos and screenshots of the Franka arm performing decluttering and shelf stocking tasks.

---

## References

1. Sutton, R. S., & Barto, A. G. (2018). *Reinforcement Learning: An Introduction*.
2. [MuJoCo Documentation](https://mujoco.org/documentation)

---

