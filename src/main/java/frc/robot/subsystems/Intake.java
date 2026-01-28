// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.TunerConstants;
import frc.robot.Constants;


/**
 * The Wrist subsystem is responsible for controlling the wrist of the robot.
 */
public class Intake extends SubsystemBase {

  // Create the possible states of the wrist
  public enum IntakeState {
    IDLE,
    MANUAL,
    POSITION,
    CLIMB,
  }

 public IntakeState intakeState = IntakeState.IDLE;


  // Create a variable to store the current state of the wrist

  // Create a talonfx for the 
  public TalonFX Intake = new TalonFX(Constants.Ports.INTAKE);



  // Create an absolute encoder for the wrist "poop" Charles
  public DutyCycleEncoder IntakeEncoder = new DutyCycleEncoder(Constants.Ports.INTAKE_ENCODER);

  // Create a PID Controller for the wrist
  public PIDController controller = new PIDController(
      Constants.Intake.Intake_KP,
      Constants.Intake.Intake_KI,
      Constants.Intake.Intake_KD);

  // Create a variable to store the setpoint of the wrist
  public static double setpointIntake = 0;

  // Create a variable to store the idle setpoint of the wrist
  public static double idleSetpoint = 3;

  // Wrist Constructor
  public Intake() {

    // Create a talonfx configurator.
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Soft Limits
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 135;	
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 150;	

    // Inverted and Neutral Mode
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply Configs to the talonfx on the wrist
		Intake.getConfigurator().apply(config);

    

    // Have Wrist Encoder Output in degrees
  }

  /** Obtain the current state of the wrist 
   * 
   * @return the current state of the wrist as a WristState
  */
  public IntakeState getState() {
    return intakeState;
  }

  /** Set the state of the wrist
   * 
   * @param state the state to set the wrist to
  */
  public void setState(IntakeState state) {
    intakeState = state;
  }

  public void setIntakePosition() {
    double output = 0;
    output = MathUtil.clamp(controller.calculate(getWristPosition(), setpointIntake), -.2, .3);
    setIntakePercent(output * 1);
  }

  /**
   * Set the percent output of the wrist talonfx with feedforward.
   * <p>
   * It will first set the percent output to the given percent.
   * <p>
   * For the gravity feedforward, it multiplies the KG constant by the cosine of the wrist angle.
   * <p>
   * Finally, it will add the KS constant to the output.
   * <p>
   * This result is the percent output that will be assigned to the wrist
   * 
   * @param percent the percent output to set the wrist to
   */
  public void setIntakePercent(double percent) {
    double output = 0;


    


    output = percent +
    Math.cos(Math.toRadians(getWristPosition())) * Constants.Intake.Intake_KG +
    Constants.Intake.Intake_KS;

   

// change the way these < > point if need be
    
    


  
  }

  /**
   * Get the current setpoint of the wrist
   * 
   * @return the current setpoint of the wrist
   */
  public static double getSetpoint() {
    return setpointIntake;
  }

  /**
   * Set the setpoint of the wrist
   * 
   * @param setpoint the setpoint to set the wrist to
   */
  public static void setSetpoint(double setpoint) {
    setpointIntake = setpoint;
  }

  /**
   * Get the current position of the wrist, accounting for the offset of the rev encoder
   * 
   * @return the current angle of the wrist in degrees
   */
  public double getWristPosition() {
    return (IntakeEncoder.get() * 360);
  }


  /**
   * Check if the wrist is at the setpoint within a given tolerance
   * @param tolerance the tolerance to check if the wrist is at the setpoint
   * @return true if the wrist is at the setpoint within the tolerance, false otherwise
   */

	public boolean isAtSetpoint(double tolerance) {
		return Math.abs((setpointIntake - getWristPosition())) <= tolerance;
	}
//the thing goes thing a thing thing
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Angle", getWristPosition());
    SmartDashboard.putString("Intake State", String.valueOf(getState()));
    SmartDashboard.putNumber("Intake Setpoint", getSetpoint());
    SmartDashboard.putNumber("Intake Output", Intake.get());
    
    idleSetpoint = getState().equals(IntakeState.IDLE) ? idleSetpoint : setpointIntake;
    
    // Wrist State Machine
    switch (intakeState) {
      
      // In the idle state, the wrist rests within the robot
      case IDLE:
        setSetpoint(0);
         setIntakePosition();
        break;

      // In the manual state, the wrist is controlled directly by the operator
      case MANUAL:
        setIntakePercent(RobotContainer.driverPad.getRightY() * 0.3);
        break;
      
      case POSITION:
         setIntakePosition();
        break;

      case CLIMB:
        setSetpoint(0);
        setIntakePosition();
        break;

      default:
        break;
        
      
    }
    
  }
}