//nothing to see here

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Hood extends SubsystemBase {

  // Create the possible states of the Hood
  public enum HoodState {
    IDLE,
    MANUAL,
    POSITION,
  }

  // Create a variable to store the current state of the hood
  public HoodState hoodState = HoodState.IDLE;

  // Create a talonfx for the hood
  public TalonFX Hood = new TalonFX(Constants.Ports.HOOD);

  // Create a PID Controller for the hood
  private PIDController controller = new PIDController(
      Constants.Hood.Hood_KP,
      Constants.Hood.Hood_KI,
      Constants.Hood.Hood_KD);

  {

  }

  // Create a variable to store the setpoint of the hood in kraken encoder ticks
  public volatile static double setpointHood  = 0;
  
    // Hood Constructor
    public Hood() {
  
      // Create a talonfx configurator.
      TalonFXConfiguration config = new TalonFXConfiguration();
  
      config.CurrentLimits.SupplyCurrentLimit = 20;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
  
      config.CurrentLimits.StatorCurrentLimit = 120;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
  
      // Soft Limits
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = -2;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.2;
  
      // Inverted and Neutral Modes
      // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  
      // Apply the configurator to the hood motor
      Hood.getConfigurator().apply(config);
    }
  
    /**
     * Get the state of the hood
     * 
     * @return The state of the hood as an HoodState
     */
    public HoodState getState() {
      return hoodState;
    }
  
    /**
     * Set the state of the hood
     * 
     * @param state The state to set the hood to
     */
    public void setState(HoodState state) {
      hoodState = state;
    }
  
    public void setHoodPosition() {
      double output = 0;
  
        output = MathUtil.clamp(controller.calculate(Hood.getPosition().getValueAsDouble(), setpointHood),
            .25, -.25) +
            Constants.Hood.Hood_KS;
      
      Hood.set(output);
    }
  
    /**
     * Set the output of the hood with feedforward
     * 
     * @param percent The percentage to set the hood to
     */
    public void setHoodPercent(double percent) {
      Hood.set(percent + Constants.Hood.Hood_KS);
    }
  
    /**
     * Get the current position of the hood
     * 
     * @return the current angle of the hood in kraken ticks
     */
    public double getHoodPosition() {
      return Hood.getPosition().getValueAsDouble() * 360;
    }
  
    /**
     * Get the current setpoint of the hood
     * 
     * @return the current setpoint of the hood
     */
    public static double getSetpoint() {
      return setpointHood;
  }

  /**
   * Set the setpoint of the hood
   * 
   * @param setpoint the setpoint to set the hood to, in kraken encoder ticks
   */
  public void setSetpoint(double setpoint) {
    setpointHood = setpoint;
  }

  /**
   * Check if the hood is at the setpoint within a given tolerance
   * 
   * @param tolerance the tolerance to check if the hood is at the setpoint
   * @return true if the hood is at the setpoint within the tolerance, false otherwise
   */
  public boolean isAtSetpoint(double tolerance) {
    return Math.abs((setpointHood - getHoodPosition())) <= tolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Encoder", getHoodPosition());
    SmartDashboard.putNumber("Hood Output", Hood.get());
    SmartDashboard.putNumber("Hood Setpoint", getSetpoint());

    // Hood State Machine
    switch (hoodState) {

      // In the Idle state, the hood rests
      case IDLE:
        setHoodPosition();
        break;

      // In the Manual state, the hood is controlled directly by the operator
      case MANUAL:
        setHoodPercent(RobotContainer.driverPad.getLeftY() * 0.3);
        break;

      // In the Position state, the hood is controlled by the setpoint
      case POSITION:
        setHoodPosition();
        break;
    }
  }
}
