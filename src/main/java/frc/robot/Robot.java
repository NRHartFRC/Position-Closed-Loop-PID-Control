// Author: N. Rombach
// Date: September 2023

// import directories as a package
package frc.robot;

// import the TimedRobot as a superclass
import edu.wpi.first.wpilibj.TimedRobot;

// import SmartDashboard for displaying 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import REV classes
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import analoginput class for pot (FOR OPTION 2 ONLY)
import edu.wpi.first.wpilibj.AnalogInput;

// primary class Robot that is a subclass of the TimedRobot superclass
public class Robot extends TimedRobot {

  // initialize and declare variables
  private static final int deviceID = 4;
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  // limit switch intitialization
  private SparkMaxLimitSwitch leftLimitSwitch;
  private SparkMaxLimitSwitch rightLimitSwitch;  

  // analog input(FOR OPTION 2 ONLY)
  private AnalogInput potentiometer;

  @Override
  public void robotInit() {

    // initialize motor controller and the motor it operates
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushed);

    // Set the continuous current limit (in amps)
    int continuousCurrentLimit = 20;
    m_motor.setSmartCurrentLimit(continuousCurrentLimit);

    // Set the peak current limit (in amps)
    int peakCurrentLimit = 25;
    m_motor.setSecondaryCurrentLimit(peakCurrentLimit);

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object (this is a member variable)
     */
    m_pidController = m_motor.getPIDController();

    // Encoder member variable/object created to display position values, use kQuadrature instead of kHallSensor
    m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4);

    // PID coefficients
    kP = 1.8; 
    kI = 0.0;
    kD = 0.1; 
    kIz = 25; 
    kFF = 0; 
    kMaxOutput = 50; 
    kMinOutput = -50;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 30);

    // potentiometer initialization (FOR OPTION 2 ONLY)
    potentiometer = new AnalogInput(0);

    // Create instances of SparkMaxLimitSwitch for left and right limit switches
    leftLimitSwitch = m_motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    rightLimitSwitch = m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    
    // Display limit switch statuses on SmartDashboard
    SmartDashboard.putBoolean("Left Limit Switch Status", !leftLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Right Limit Switch Status", !rightLimitSwitch.isPressed());    
  }

  @Override
  public void teleopPeriodic() {

    // monitor motor speed (RPM)
    double motorSpeed = m_encoder.getVelocity();
    SmartDashboard.putNumber("Motor Speed (RPM)", motorSpeed);

    // monitor motor current (A)
    double motorCurrent = m_motor.getOutputCurrent();
    SmartDashboard.putNumber("Motor Current (A)", motorCurrent);

    // monitor motor output (Duty Cycle)
    double motorOutput = m_motor.getAppliedOutput();
    SmartDashboard.putNumber("Motor Output (DC)", motorOutput);

    // potentiometer captures analog input value (FOR OPTION 2 ONLY)
    double potentiometerValue = potentiometer.getVoltage();

    // mapped voltage 0-5V is passed to a setpoint variable for position control (FOR OPTION 2 ONLY)
    double analogsetpoint = mapPotentiometerValueToSetpoint(potentiometerValue);

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 10); //SETPOINT


    // apply supersoft limits to the 'rotations' variable (the setpoint) to prevent singularity
    rotations = Math.max(10, Math.min(400, rotations));

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    // Option 1 - Manual Input
    // PIDController object is commanded to a rotation value using the SetReference() method for user-entered input
    m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition); // <--- CODE!

    // Option 2 - Analog Input
    // PIDController object is commanded to a setpoint using the SetReference() method for analog input
    // m_pidController.setReference(analogsetpoint, CANSparkMax.ControlType.kPosition); // <--- CODE!

    // print debugging information to console (FRC Driver Station: Click gear icon > View console)
    System.out.println("kP SD: " + p);
    System.out.println("Speed RPM: " + motorSpeed);
    System.out.println("Motor Current (A): " + motorCurrent);
    System.out.println("SetPoint: " + rotations);
    System.out.println("ProcessVariable: " + m_encoder.getPosition());
    
    // Option 1
    // display to SmartDashboard (View > Editable (or ctrl+e), right click object and 'change to...' as needed)
    SmartDashboard.putNumber("SetPoint Ref.", rotations);
    // show the process variable for the user-entered input
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());

    // Option 2
    // Update the "Mapped Setpoint" value on the SmartDashboard
    SmartDashboard.putNumber("Mapped Setpoint", analogsetpoint);
    // show the process variable for the analog input
    SmartDashboard.putNumber("Analog Setpoint", m_encoder.getPosition());

    // Check the status of the left limit switch
    boolean isLeftLimitSwitchClosed = leftLimitSwitch.isPressed();

    // Check the status of the right limit switch
    boolean isRightLimitSwitchClosed = rightLimitSwitch.isPressed();

    // Display the limit switch statuses on SmartDashboard
    SmartDashboard.putBoolean("Left Limit Switch Status", isLeftLimitSwitchClosed);
    SmartDashboard.putBoolean("Right Limit Switch Status", isRightLimitSwitchClosed);

    // Use the status as needed in your code
    if (isLeftLimitSwitchClosed) {
        // The left limit switch is closed (triggered)
        // Perform actions accordingly
    } else {
        // The left limit switch is open
        // Perform other actions
    }

    if (isRightLimitSwitchClosed) {
        // The right limit switch is closed (triggered)
        // Perform actions accordingly
    } else {
        // The right limit switch is open
        // Perform other actions
    }
  }

  private double mapPotentiometerValueToSetpoint(double potentiometerValue) {
    // Implement the mapping based on potentiometer and mechanism
    // Example: Assuming potentiometer voltage ranges from 0 to 5V and
    // we want a setpoint between 10 and 400 rotations:
    
    double minVoltage = 0.0;
    double maxVoltage = 5.0;
    double minSetpoint = 10.0;
    double maxSetpoint = 400.0;
    
    // linearly map the potentiometer value to the setpoint
    double mappedSetpoint = (potentiometerValue - minVoltage) / (maxVoltage - minVoltage) * (maxSetpoint - minSetpoint) + minSetpoint;
    
    return mappedSetpoint;
  }
}
