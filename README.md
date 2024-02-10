# Pick and Place Robotic Arm

## Overview
This repository contains the source code, configuration files, and documentation for a pick and place robotic arm simulation developed using ROS2 (Robot Operating System 2) and Gazebo simulator.

<!-- ![Robotic Arm in Gazebo](docs/images/robotic_arm_gazebo.png) -->

## Features
- **ROS2-Based Control:** Utilizes the power of ROS2 for modular and distributed robotic system development.
- **Gazebo Simulation:** Realistic robotic arm simulation in the Gazebo environment for testing and validation.
- **Pick and Place Operation:** Demonstrates a basic pick and place functionality.
- **Modular Design:** Easily extendable for integration with other robotic components and systems.

### Prerequisites

Make sure you have the following installed:
- Python 3.6 or higher
- ROS2 Foxy Fitzroy or later installed on your system. [ROS2 Installation Guide](https://docs.ros.org/en/foxy/Installation.html)
- Gazebo simulator. [Gazebo Installation Guide](http://gazebosim.org/tutorials?tut=install_ubuntu)

## Installation
1. Clone this repository to your local machine:

    ```bash
    git clone https://github.com/Huseny/Pick-and-Place-Robotic-Arm.git
    ```

2. Build the ROS2 workspace:

    ```bash
    cd robotic-arm
    colcon build
    ```

## Usage
1. Source the ROS2 setup file:

    ```bash
    source install/setup.bash
    ```

2. Launch the robotic arm simulation in Gazebo:

    ```bash
    ros2 launch robotic_arm gazebo.launch.py
    ```

3. Open a new terminal and run the pick and place controller:

    ```bash
    ros2 run robotic_arm controller.launch.py
    ```

<!-- ## Documentation
Detailed documentation, including system architecture, configuration details, and usage guidelines, can be found in the [docs](docs) directory. -->

## Contributing
Contributions to enhance the system or address issues are welcome. Please fork the repository, create a branch, commit your changes, and submit a pull request:

1. **Fork the project.**
2. **Create a new branch:**
    ```bash
    git checkout -b feature-branch
    ```
3. **Make your changes and commit them:**
    ```bash
    git commit -m 'Add new feature'
    ```
4. **Push to the branch:**
    ```bash
    git push origin feature-branch
    ```
5. **Submit a pull request.**

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
<!-- 
## Acknowledgments
- Mention any libraries, frameworks, or tools used in the development. -->

## Contact
For any inquiries or issues, please contact [husenyusuf876@gmail.com].

