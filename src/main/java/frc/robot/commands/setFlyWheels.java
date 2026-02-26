// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Flywheel.FlywheelState;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setFlyWheels extends Command {
  /** Creates a new SetTurretManualOverride. */
  public setFlyWheels() {
    addRequirements(RobotContainer.flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.flywheel.setState(FlywheelState.SHOOTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.flywheel.setState(FlywheelState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}