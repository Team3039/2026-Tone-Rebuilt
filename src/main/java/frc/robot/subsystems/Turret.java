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

public class Turret extends SubsystemBase {

	// Create the possible states of the Turret
	public enum TurretState {
		IDLE,
		MANUAL,
		POSITION,

	}



	// Create a variable to store the current state of the turret
	public TurretState turretState = TurretState.IDLE;

	// Create a talonfx for the turret
	public TalonFX Turret = new TalonFX(Constants.Ports.TURRET);

	// Create a PID Controller for the turret
	private PIDController controller = new PIDController(
			Constants.Turret.Turret_KP,
			Constants.Turret.Turret_KI,
			Constants.Turret.Turret_KD);

	{

	}


	// Create a variable to store the setpoint of the elevator in kraken encoder
	// ticks
	public static double setpointTurret  = 0;

	// Turret Constructor
	public Turret() {

		// Create a talonfx configurator.
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.CurrentLimits.SupplyCurrentLimit = 20;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;

		config.CurrentLimits.StatorCurrentLimit = 120;
		config.CurrentLimits.StatorCurrentLimitEnable = true;

		// Soft Limits
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -50;

		// Inverted and Neutral Modes
		// config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// Apply the configurator to the elevator motor
		Turret.getConfigurator().apply(config);
	}

	/**
	 * Get the state of the turret
	 * 
	 * @return The state of the turret as an TurretState
	 */
	public TurretState getState() {
		return turretState;
	}

	/**
	 * Set the state of the turret
	 * 
	 * @param state The state to set the turret to
	 */
	public void setState(TurretState state) {
		turretState = state;
	}

	
    


	public void setTurretPosition() {
		double output = 0;

		output = MathUtil.clamp(controller.calculate(Turret.getPosition().getValueAsDouble(), setpointTurret ),
				.05 ,-.05) +
				Constants.Turret.Turret_KS;

		Turret.set(output);
	
        
	}

	/**
	 * Set the output of the elevator with feedforward
	 * 
	 * @param percent The percentage to set the turret to
	 */
	public void setTurretPercent(double percent) {
		Turret.set(percent + Constants.Turret.Turret_KS);
	}

	public void stop() {
        Turret.set(0);
	}

	/**
	 * Get the current position of the turret
	 * 
	 * @return the current angle of the turret in kraken ticks
	 */
	public double getTurretPosition() {
		double position = Turret.getPosition().getValueAsDouble() + 0.0244140625;

		return position * 34.83651483651484;
	}

	/**
	 * Get the current setpoint of the elevator
	 * 
	 * @return the current setpoint of the elevator
	 */
	public static double getSetpoint() {
		return setpointTurret ;
	}

	/**
	 * Set the setpoint of the elevator
	 * 
	 * @param setpoint the setpoint to set the elevator to, in kraken encoder ticks
	 */
	public static void setSetpoint(double setpoint) {
		setpointTurret = setpoint;
	}

	
	/**
	 * Check if the elevator is at the setpoint within a given tolerance
	 * 
	 * @param tolerance the tolerance to check if the elevator is at the setpoint
	 * @return true if the wrist is at the setpoint within the tolerance, false
	 *         otherwise
	 */
	public boolean isAtSetpoint(double tolerance) {
		return Math.abs((setpointTurret - getTurretPosition())) <= tolerance;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Turret Encoder", getTurretPosition());
		SmartDashboard.putNumber("Turret Output", Turret.get());
		// SmartDashboard.putNumber("Turret Output Current",
		// Turret.getSupplyCurrent().getValueAsDouble());
		// SmartDashboard.putString("Turret State", String.valueOf(getState()));
		SmartDashboard.putNumber("Turret Setpoint", getSetpoint());

		// Turret State Machine
		switch (turretState) {

			// In the Idle state, the elevator rests at the bottom of the robot
			case IDLE:
				stop();
				break;

			// In the Manual state, the elevator is controlled directly by the operator
			case MANUAL:
				setTurretPercent(RobotContainer.driverPad.getLeftY() * 0.3);
				break;

			// In the Position state, the elevator is controlled by the setpoint
			case POSITION:
				setTurretPosition();
				break;
		}
	}
}