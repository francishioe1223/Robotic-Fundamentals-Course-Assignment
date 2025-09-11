# Robot Kinematics & Motion Planning: Serial & Parallel Manipulator Analysis

This repository contains my comprehensive coursework for the **Robotic Fundamentals** module in the Robotics MSc program at the University of Bristol. The coursework demonstrates theoretical understanding and practical implementation of fundamental robotics concepts including kinematics, dynamics, and control systems for serial and parallel manipulators.

## Project Overview

This assignment demonstrates mastery of core robotics concepts through three main parts:

- **Part I**: Serial Manipulator Kinematics and Control
- **Part II**: Advanced Kinematics and Trajectory Planning  
- **Part III**: Parallel Robot Forward Kinematics

## Project Structure

```
├── README.md                           # This file
├── CW-RF.pdf                          # Assignment requirements and specifications
├── PartI/                             # Serial Manipulator Analysis
│   ├── A1.mlx                         # DH Parameters and Forward Kinematics
│   ├── PartI_A2.mlx                   # Workspace Analysis and Visualization
│   ├── PartI_A3.mlx                   # Jacobian Analysis and Singularities
│   ├── B1.mlx                         # Inverse Kinematics Implementation
│   ├── B2.mlx                         # Joint Space Trajectory Planning
│   ├── PartI_B3.mlx                   # Obstacle Avoidance and Path Planning
│   ├── FK_LynxmotionRobot.m           # Forward kinematics function
│   ├── IK_LynxmotionRobot.m           # Inverse kinematics function
│   ├── IK_LynxmotionRobot_WithObstacle.m # Obstacle-aware inverse kinematics
│   ├── DistalDH.m                     # DH transformation matrix function
│   ├── Calc_JointPos.m                # Joint position calculation
│   └── inverse_kinematics.m           # General inverse kinematics solver
├── PartII/                            # Advanced Kinematics
│   ├── PartII_1.mlx                   # Velocity Kinematics and Jacobian
│   ├── PartII_2.mlx                   # Trajectory Generation and Control
│   └── inverse_kinematics.m           # Enhanced IK solver
└── PartIII/                           # Parallel Robotics
    └── Forward kinematics for parallel robot.mlx # Parallel robot FK analysis
```

## Part I: Serial Manipulator Kinematics and Control

**Objective:** Analyze and implement kinematics solutions for a 5-DOF Lynxmotion robotic arm using Denavit-Hartenberg parameters.

### Technical Implementation
- **DH Parameters**: Complete kinematic modeling using modified DH convention
- **Forward Kinematics**: End-effector position and orientation calculation
- **Inverse Kinematics**: Joint angle solutions for desired end-effector poses
- **Workspace Analysis**: Reachable workspace visualization and boundary analysis
- **Jacobian Analysis**: Velocity relationships and singularity detection
- **Obstacle Avoidance**: Path planning with collision detection

### Key Features
- **Robot Specifications**: 5-DOF Lynxmotion arm (d1=3, L1=10, L2=10, L3=3)
- **Analytical Solutions**: Closed-form inverse kinematics with multiple solutions
- **Workspace Visualization**: 3D plotting of reachable workspace boundaries
- **Singularity Analysis**: Identification and visualization of singular configurations
- **Trajectory Planning**: Joint space and Cartesian space path planning
- **Collision Detection**: Obstacle avoidance algorithms for safe motion planning

### Functions Implemented
- `FK_LynxmotionRobot()`: Forward kinematics solver
- `IK_LynxmotionRobot()`: Inverse kinematics solver  
- `IK_LynxmotionRobot_WithObstacle()`: Collision-aware inverse kinematics
- `DistalDH()`: DH transformation matrix computation
- `Calc_JointPos()`: Joint position calculation for visualization

## Part II: Advanced Kinematics and Trajectory Planning

**Objective:** Implement advanced kinematic analysis including velocity kinematics, Jacobian computation, and trajectory generation algorithms.

### Technical Implementation
- **Velocity Kinematics**: Relationship between joint velocities and end-effector velocity
- **Jacobian Matrix**: Analytical computation and numerical verification
- **Trajectory Generation**: Smooth trajectory planning in joint and Cartesian space
- **Motion Control**: Implementation of trajectory following algorithms

### Key Features
- **Differential Kinematics**: Velocity and acceleration analysis
- **Jacobian Singularities**: Detection and handling of kinematic singularities
- **Trajectory Optimization**: Time-optimal and smooth trajectory generation
- **Real-time Control**: Implementation of control algorithms for trajectory tracking

## Part III: Parallel Robot Forward Kinematics

**Objective:** Analyze forward kinematics of parallel manipulators using numerical methods and constraint equations.

### Technical Implementation
- **Parallel Kinematics**: Forward kinematics solution for parallel mechanisms
- **Constraint Equations**: Mathematical modeling of kinematic constraints  
- **Numerical Methods**: Iterative solutions for complex kinematic equations
- **Multiple Solutions**: Handling of multiple forward kinematic solutions

### Key Features
- **Parallel Architecture**: Analysis of closed-loop kinematic chains
- **Forward Kinematics**: Numerical solution of constraint equations
- **Configuration Analysis**: Multiple assembly modes and workspace analysis

## Technical Skills Demonstrated

### Mathematical Modeling
- **Denavit-Hartenberg Parameters**: Systematic kinematic modeling
- **Transformation Matrices**: Homogeneous transformations and coordinate frames
- **Analytical Solutions**: Closed-form inverse kinematics derivation
- **Numerical Methods**: Iterative algorithms for complex kinematic problems

### MATLAB Programming
- **Live Scripts**: Interactive analysis and visualization
- **Function Development**: Modular and reusable kinematic functions
- **3D Visualization**: Workspace plotting and robot animation
- **Algorithm Implementation**: Trajectory planning and optimization algorithms

### Robotics Concepts
- **Serial Manipulators**: Complete kinematic analysis of open-chain robots
- **Parallel Robots**: Forward kinematics of closed-loop mechanisms
- **Workspace Analysis**: Reachable space computation and visualization
- **Singularity Analysis**: Identification of degenerate configurations
- **Motion Planning**: Trajectory generation and obstacle avoidance

## Key Results Summary

| Part | Analysis | Key Achievement | Performance |
|------|----------|----------------|-------------|
| I-A | Forward Kinematics | Complete DH model | Accurate end-effector positioning |
| I-A | Workspace Analysis | 3D visualization | Complete reachable space mapping |
| I-A | Jacobian Analysis | Singularity detection | Identified critical configurations |
| I-B | Inverse Kinematics | Analytical solution | Multiple solution handling |
| I-B | Trajectory Planning | Smooth motion | Optimized joint trajectories |
| I-B | Obstacle Avoidance | Collision-free paths | Safe motion planning |
| II | Velocity Kinematics | Jacobian computation | Differential motion analysis |
| II | Trajectory Generation | Advanced planning | Time-optimal trajectories |
| III | Parallel Kinematics | Forward solution | Numerical constraint solving |

## Learning Outcomes

This project demonstrates mastery of:

1. **Kinematic Modeling**: DH parameters and transformation matrices for robot modeling
2. **Forward Kinematics**: End-effector pose calculation from joint parameters  
3. **Inverse Kinematics**: Joint angle computation for desired end-effector poses
4. **Workspace Analysis**: Reachable space computation and boundary visualization
5. **Jacobian Analysis**: Velocity relationships and singularity identification
6. **Motion Planning**: Trajectory generation and obstacle avoidance algorithms
7. **Parallel Robotics**: Forward kinematics of closed-loop mechanisms
8. **MATLAB Programming**: Advanced programming for robotics applications

## Hardware Specifications

### Lynxmotion 5-DOF Robotic Arm
- **Joint 1**: Base rotation (θ₁)
- **Joint 2**: Shoulder pitch (θ₂) 
- **Joint 3**: Elbow pitch (θ₃)
- **Joint 4**: Wrist pitch (θ₄)
- **Joint 5**: Wrist roll (θ₅)

### Physical Parameters
- **Base height**: d₁ = 3 units
- **Upper arm length**: L₁ = 10 units  
- **Forearm length**: L₂ = 10 units
- **End-effector offset**: L₃ = 3 units

## Getting Started

### Prerequisites
- MATLAB R2020b or later
- Robotics System Toolbox 
- Symbolic Math Toolbox

### Running the Code
1. **Part I**: Open MATLAB live scripts in `PartI/`
   - Start with `A1.mlx` for DH parameters and forward kinematics
   - Progress through `PartI_A2.mlx` and `PartI_A3.mlx` for workspace and Jacobian analysis
   - Explore inverse kinematics with `B1.mlx`, `B2.mlx`, and `PartI_B3.mlx`
2. **Part II**: Execute `PartII_1.mlx` and `PartII_2.mlx` for advanced kinematics
3. **Part III**: Run `Forward kinematics for parallel robot.mlx` for parallel robot analysis

### Function Usage
```matlab
% Forward Kinematics Example
[x, y, z, psi] = FK_LynxmotionRobot(0, 30, -60, 45, 0);

% Inverse Kinematics Example  
[t1, t2, t3, t4, t5] = IK_LynxmotionRobot(15, 0, 8, pi/4);

% Joint Position Calculation
joint_pos = Calc_JointPos([0, 30, -60, 45, 0]);
```

## Theoretical Foundation

This coursework builds upon fundamental robotics concepts including:
- **Denavit-Hartenberg Convention**: Systematic approach to kinematic modeling
- **Homogeneous Transformations**: Mathematical representation of rigid body motion
- **Analytical Geometry**: Geometric solutions for inverse kinematics
- **Linear Algebra**: Matrix operations for transformation and Jacobian computation
- **Numerical Methods**: Iterative algorithms for complex kinematic problems
- **Optimization Theory**: Trajectory planning and motion optimization

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---
