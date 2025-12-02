# **Example Subsystem Code**

This repository contains example code for multiple subsystems that follow the **FRC 1792 RTR Coding Standard**. It is designed to serve as a reference for implementing various subsystems in a robot project.

---

## **Features**
- **Subsystem Examples**: Includes examples for intake, arm, and elevator.
- **Command-Based Framework**: Demonstrates the use of WPILib's command-based programming model.
- **Sensor Integration**: Examples include subsystems with and without sensor feedback.
- **Motion Control**: Implements advanced control techniques like Motion Magic and Dynamic Motion Magic for precise movement.

---

## **Project Structure**
### **Subsystems**
Reusable subsystem classes for different robot mechanisms:
- **Arm**:
  - **TalonFX_Arm_CANCoder_MM**: Controls a pivoting arm using a TalonFX motor controller with Motion Magic. The arm position is measured using a CANcoder through-bore encoder. Includes methods for moving the arm up, down, stopping, and setting specific positions.
- **Intake**:
  - **TalonFX_Intake_WithoutSensor**: Manages the intake rollers without sensor feedback. Includes methods for intaking, outtaking, and stopping the rollers.
  - **TalonFX_Intake_WithSensor**: Manages the intake rollers with sensor feedback. The sensor stops the rollers when an object is detected. Includes methods for intaking, outtaking, and stopping the rollers.
- **Elevator**:
  - **TalonFX_Elevator_MM**: Controls an elevator mechanism using two TalonFX motor controllers with Motion Magic. Includes methods for moving the elevator up, down, stopping, and setting specific positions. Includes limit switch handling for zeroing the elevator position.
  - **TalonFX_Elevator_DMM**: Similar to `TalonFX_Elevator_MM` but uses Dynamic Motion Magic for more advanced motion control. Includes limit switch handling for zeroing the elevator position.

---

### **Commands**
Commands for controlling subsystems with and without sensors:
- **Arm_Commands**:
  - **Arm_Setpoint_CMD**: Sets the arm to a specific setpoint using Motion Magic. The setpoint is defined in units based on the sensor-to-mechanism ratio.
- **Intake_Commands**:
  - **Intake_WithoutSensor**:
    - **Roller_Intake_CMD**: Activates the intake rollers to collect game pieces without sensor feedback.
    - **Roller_Outtake_CMD**: Activates the intake rollers to release game pieces without sensor feedback.
  - **Intake_WithSensor**:
    - **Roller_Intake_WithSensor_CMD**: Activates the intake rollers to collect game pieces. The rollers stop automatically when the sensor detects an object.
    - **Roller_Outtake_WithSensor_CMD**: Activates the intake rollers to release game pieces. This command does not rely on sensor feedback.
- **Elevator_Commands**:
  - **Elevator_MM_Setpoint_CMD**: Moves the elevator to a specific setpoint using Motion Magic. The setpoint is defined in rotations and can be adjusted dynamically.
  - **Elevator_DMM_Setpoint_CMD**: Moves the elevator to a specific setpoint using Dynamic Motion Magic. This allows for more advanced motion control and dynamic adjustments.

---

## **Getting Started**
1. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/Example-Subsystem-Code.git