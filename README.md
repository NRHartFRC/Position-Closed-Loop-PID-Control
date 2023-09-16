# Position Closed Loop Control

### Description
This example shows how to use a PID closed loop controller to set the position of a motor to a specified number of rotations (variable is called rotations, really it is a setpoint on a linear scale).

### Usage
Set the variables at the beginning of the example to match your setup.
- `deviceID` - CAN device ID

## **Instructions:**
- open WPILib VSCode application
- File > Open Folder...Select and open the root folder (Position Closed Loop PID Control)
- wiring connections (power(s), CAN bus, breakout board connections)
- deploy: right click build.gradle file, then click Deploy Robot Code
- terminal will appar and confirm BUILD SUCCESSFUL
- open FRC Driver StationOpen Console: click gear icon > View Console
- open SmartDashboard 2023 application (comes with WPILib Installer for Windows)
- Enable - FRC Driver Station
- manually enter Set Rotations in SmartDashboard application
- off-clicking from the SmartDashboard window will execute changes

**Note:** PID Coefficients can be adjusted on SmartDashboard as well as the set rotations (position setpoint). Off-clicking from SmartDashbaord executes the changes made. The setpoint and process variable will be displayed on SmartDashboard.

## Improvements
- configure limit switches
- make motor go faster


