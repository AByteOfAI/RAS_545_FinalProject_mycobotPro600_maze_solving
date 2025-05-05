# RAS_545_FinalProject_mycobotPro600_maze_solving
# MyCobot Pro 600 Maze Solver

## Project Overview

This repository contains the code and resources for a project developed for the RAS545: Robotic Systems 1 course at Arizona State University. The project enables an Elephant Robotics MyCobot Pro 600 collaborative robot arm to autonomously solve a 4x4 rectangular maze using computer vision, path planning, digital twin simulation, and real-time control.

**Video Demonstration:** [Link to Project Video (Google Drive)](https://drive.google.com/drive/folders/1z-Yw3Gj1HG1zU_HUEZpT56SMKt-uV438?usp=drive_link)

## Features

* **Automated Maze Acquisition:** Captures the maze layout using an AI-Kit camera via OpenCV.
* **Image Processing:** Automatically crops, rotates, and identifies start/end points (red/green blobs) in the maze image.
* **Optimal Pathfinding:** Uses Dijkstra's algorithm to calculate the shortest path through the maze.
* **Coordinate Transformation:** Converts image pixel coordinates to the robot's real-world workspace coordinates.
* **Kinematic Modeling:** Utilizes MATLAB and the Robotics System Toolbox to model the MyCobot Pro 600.
* **Inverse Kinematics:** Calculates the necessary joint angles for the robot to follow the planned path.
* **Digital Twin Simulation:** Employs a MATLAB/Simscape Multibody digital twin to rigorously simulate and validate the robot's trajectory, checking for reachability and collisions *before* physical execution.
* **Real-Time Robot Control:** Streams joint angle commands to the physical MyCobot Pro 600 via TCP/IP sockets using Python for smooth execution.
* **Validation:** Compares simulated results with real-world execution data.

## Workflow

1.  **Setup:** Position the maze within the camera's view.
2.  **Capture:** Run the Python script to capture and process the maze image using OpenCV.
3.  **Solve & Plan:** The system identifies start/end points, solves the maze using Dijkstra's, and generates waypoints.
4.  **Transform & Densify:** Waypoints are converted to robot coordinates and intermediate points are added for smoothness.
5.  **IK Calculation:** MATLAB calculates the required joint angles using inverse kinematics.
6.  **Simulate:** The trajectory is validated in the MATLAB/Simscape digital twin.
7.  **Execute:** The Python script streams the validated joint angles to the MyCobot Pro 600 via TCP/IP for physical execution.

## Technologies Used

* **Hardware:**
    * Elephant Robotics MyCobot Pro 600
    * AI-Kit Camera
* **Software & Libraries:**
    * **Python:** Main control scripts, socket communication.
    * **OpenCV:** Image acquisition, processing.
    * **MATLAB:**
        * Robotics System Toolbox: Robot modeling, FK/IK.
        * Simscape Multibody: Digital twin simulation.
    * **Algorithms:** Dijkstra's Algorithm.
* **Communication Protocol:** TCP/IP.

## Setup & Installation

*(Note: Specific setup steps would depend on the exact code structure. Add details here based on your repository.)*

1.  **Hardware Setup:**
    * Connect the MyCobot Pro 600 and AI-Kit camera to the control computer.
    * Ensure network connectivity for TCP/IP communication.
2.  **Software Prerequisites:**
    * Install Python and required libraries (`opencv-python`, etc.).
    * Install MATLAB with Robotics System Toolbox and Simscape Multibody.
3.  **Configuration:**
    * Calibrate the camera and update coordinate transformation parameters if necessary.
    * Configure TCP/IP settings (IP address, port) in both Python and MATLAB scripts/robot settings.
4.  **Clone Repository:**
    ```bash
    git clone [https://github.com/AByteOfAI/RAS_545_FinalProject_mycobotPro600_maze_solving.git](https://github.com/AByteOfAI/RAS_545_FinalProject_mycobotPro600_maze_solving.git)
    cd RAS_545_FinalProject_mycobotPro600_maze_solving
    ```

## Acknowledgements

* Dr. Mostafa Yourdkhani (Instructor, RAS545)
* Rajdeep Adak (Lab Incharge, RAS545)
* Arizona State University

## License

This project is licensed under the MIT License - see the LICENSE.md file for details.
