// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Flywheel extends SubsystemBase {
	Timer timer = new Timer();

	public static double targetVelocity = 10;
	public static double setpointAmp = 0;
	boolean isAtSetpoint;

	public enum FlywheelState {
		IDLE,
		CLOSESHOT,
		MANUAL,
		AMP,
		SOURCE,
		FEEDING,
		CLIMBING
	}

	public TalonFX shooterLeft = new TalonFX(Constants.Ports.FLYWHEEL_LEFT);
	public TalonFX shooterRight = new TalonFX(Constants.Ports.FLYWHEEL_RIGHT);



	public FlywheelState flywheelState = FlywheelState.IDLE;

	VelocityVoltage voltageLeft = new VelocityVoltage(0);
	VelocityVoltage voltageRight = new VelocityVoltage(0);

	public Flywheel() {
		CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs().withStatorCurrentLimit(70);


		shooterLeft.setNeutralMode(NeutralModeValue.Coast);
		shooterRight.setNeutralMode(NeutralModeValue.Coast);

		Slot0Configs configs = new Slot0Configs()
				.withKP(Constants.Flywheel.Flywheel_KP)
				.withKI(Constants.Flywheel.Flywheel_KI)
				.withKD(Constants.Flywheel.Flywheel_KD);

		shooterLeft.getConfigurator().apply(configs);
		shooterLeft.getConfigurator().apply(currentLimit);
		shooterRight.getConfigurator().apply(configs);
		shooterRight.getConfigurator().apply(currentLimit);


// this does not work, I dont know why I will fix later
		shooterLeft.setInverted(true);
		shooterRight.setInverted(false);



		shooterLeft.getPosition().setUpdateFrequency(0);
		shooterRight.getPosition().setUpdateFrequency(0);
		shooterRight.setControl(new StrictFollower(shooterLeft.getDeviceID()));
		

	}

	public void setState(FlywheelState state) {
		flywheelState = state;
	}

	public FlywheelState getState() {
		return flywheelState;
	}

	


	public void setWheelSpeed(double speed) {
		shooterLeft.set(speed);
	}

	public void setShooterVelocity(double RPS) {
		shooterLeft.setControl(voltageLeft.withVelocity(RPS).withFeedForward(Constants.Flywheel.Flywheel_FF));
	}

	public boolean isAtVelocitySetpoint(){
	return targetVelocity < shooterLeft.getRotorVelocity().getValueAsDouble();
	}

	@Override
	public void periodic() {
		timer.start();
		// System.out.println(isAtVelocitySetpoint());
		SmartDashboard.putNumber("RPS Shooter", shooterLeft.getRotorVelocity().getValueAsDouble());
		SmartDashboard.putBoolean("Shooter At Setpoint", shooterLeft.getRotorVelocity().getValueAsDouble() >= targetVelocity);

		SmartDashboard.putNumber("Current Output Left Shooter", shooterRight.getTorqueCurrent().getValueAsDouble());

		SmartDashboard.putString("Flywheel State", String.valueOf(getState()));

	

		// SmartDashboard.putNumber("Amper Setpoint", getSetpointAmp());

		// if (RobotState.isTeleop() && RobotState.isEnabled() && Vision.getDistanceToSpeaker() < 8){
		// 	shooterState = ShooterState.PASSIVE;
		// }

		switch (flywheelState) {
			case IDLE:
				
						setWheelSpeed(0);
				
				break;
			case CLOSESHOT:

                                                                            // I will add get distance to hub later
				  if(DriverStation.getAlliance().isPresent() && Limelight.getDistanceToHub() < 2.2){
					targetVelocity = 75;
					setShooterVelocity(150);
						}
				  else{
				targetVelocity = 80;
				setShooterVelocity(150);	
			}
				
				break;
			
			case CLIMBING:
				setWheelSpeed(0);
                break;
			case SOURCE:
				setShooterVelocity(-100);            
				break;

			case FEEDING:
			setShooterVelocity(35);
				break;
		}
	}
}