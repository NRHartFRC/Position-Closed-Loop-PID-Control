# Position Closed Loop PID Control
![PositionPIDControlTestbench](images/PositionPIDControlTestbench.jpg)  
<figcaption align = "left"><b>Figure 1: Coach Werner's testbench with RoboRio, Sparkmax, potentiometer, and JE-PLG-410</b></figcaption><br>

# Description
This program exhibits a closed loop PID controller to scontrol the position of a motor. There are two different modes/options: manual mode, and analog mode. Manual mode

# Usage
Set the variables at the beginning of the example to match your setup.
- `deviceID` - CAN device ID
- `MotorType` - kBrushed or kBrushless
- `SparkMaxRelativeEncoder.Type.kQuadrature` - KQuadrature or kHallSensor
- `countsPerRev` - 4
- `AnalogInput` - channel 0

# Walk Through
## Code Specifications
- M. Werner's testbench setup
- VS Code (w/ WPILib Extensions)
- SmartDashboard 2023 (an application for widgets, vs. Shuffleboard plugin)
- RoboRio
- CAN bus
- PDP
- Ethernet/USB/Radio
- 12V DC power supply
- REV SparkMax
- REV breakout board
- Brushed DC motor with gearbox and Hall Sensor: [JE-PLG-410](https://www.andymark.com/products/johnson-electric-plg-gearmotor-and-output-shaft)
- encoder (*RelativeEncoder library--specify kQuadrature in lieu of kHallSensor*)
- 2X normally closed limit switches
- potentiometer (OPTION 2, analog mode only)
- 3X jumper wires  (OPTION 2, analog mode only)

## **Instructions**
- download ZIP file or clone this repository
- open WPILib VSCode application
- File > Open Folder...
- select and open this root folder (Position Closed Loop PID Control)
- wiring connections (power, CAN bus, breakout board connections, connect potentiometer to analog input 0, limit switches)
- deploy: right click build.gradle file > click Deploy Robot Code
- terminal will appear and confirm BUILD SUCCESSFUL
- open FRC Driver Station
- console: click gear icon > View Console
- open SmartDashboard 2023 application (comes with WPILib Installer for Windows)
- click big button @ FRC Driver Station (disable if needed)
- Return to smart dashboard and make the widgets/objects viewable: View > Editable (toggle as needed) and View > Edit Subsystems
- To get graphs, in Editable state, right click widget Change to... > LinePlot
- Enable and rotate pot (this is Option 2)
- Update values as desired (P, I, D, Iz, FF, Max Output, Min Output)

## **Notes**
1. Linear 0-5V voltage mapping does not actually reach limit switches (reference maxSetpoint and minSetpoint, see Note 2).
2. Modify `mapPotentiometerValueToSetpoint` method at bottom of code; widen range for actual limit switch functionality.
3. Also would need to modify 'supersoft' limits (Math.max(10, Math.min(400, rotations)) to use limit switches.
4. Power is only truly killed while both limit switches are pressed.
5. USE EITHER OPTION 1 OR OPTION 2 (manual position control or analog input position control). Do not use both.
6. Comment one unused option as needed. They appear as m_pidController.setReference(); (this is the Position PID in action).
7. `rotations` variable is only used for Option 1.
8. `analogsetpoint` variable is only used for Option 2.

## Outputs 
PID Coefficients can be adjusted on SmartDashboard to tune the controller. Max Output and Min Output values can be adjusted in either in either mode--this controls the speed and power of the output (consider the PID will modulate this raw value). SmartDashboard actively monitors: (i) left & right limit switch statuses; (ii) motor current; (iii) motor speed, and (iv) motor output duty cycle.

For manual mode, use 'Set Rotations' to change position of the shuttle or whatever else you want to control by typing in a value. Off-clicking from SmartDashboard executes the changes made for manual mode. The SetPoint Ref. and ProcessVariable will be displayed on SmartDashboard as shown in Figure 2.  
![ManualMode](images/ManualMode.PNG)
<figcaption align = "left"><b>Figure 2: Manual mode from SmartDashboard</b></figcaption><br><br>

For analog option, the potentiometer completely controls the shuttle as shown in Figure 3.   
![AnalogMode](images/AnalogMode.PNG)
<figcaption align = "left"><b>Figure 3: Analog mode from SmartDashboard</b></figcaption><br>

## Future
- Sendable chooser for Option 1 (manual) and Option 2 (analog input) instead of commenting and uncommenting options
- Migrate to separate commands for integration purposes


