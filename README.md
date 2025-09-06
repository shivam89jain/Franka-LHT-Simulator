# FrankaLHT Simulator

This repository contains the **FrankaLHT Simulator**, a lightweight simulation environment for the Franka Emika Panda robot, developed at **IIT Delhi**.  
The simulator is built on top of [PyBullet](https://pybullet.org/wordpress/) and provides simple keyboard-based controls for interacting with the robot.

---

## Features
- Load a simulated Franka Panda robot arm in PyBullet.
- Control robot joints interactively using the keyboard.
- Record joint trajectories for later use in training, imitation learning, or policy development.
- **Includes 180 pre-recorded trajectories** across a variety of tabletop manipulation tasks (available in the `Trajectory/` folder).
- Lightweight, easy-to-use, and extendable.

---

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/<your-username>/FrankaLHT-Simulator.git
   cd FrankaLHT-Simulator
