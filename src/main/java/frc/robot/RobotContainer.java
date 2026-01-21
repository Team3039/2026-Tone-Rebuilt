// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
// import edu.wpi.first.units.Units;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;


public class RobotContainer {
  

// private static final Logger logger = Logger.getLogger(RobotContainer.class.getName());

private final SendableChooser<Command> autoChooser;


// APTarget ApReefF = new APTarget(TunerConstants.POSES.REEF_F)
//   .withEntryAngle(Rotation2d.kZero);


public RobotContainer() {





     autoChooser = AutoBuilder.buildAutoChooser(); //Auto chooser
    SmartDashboard.putData("Auto Chooser", autoChooser);

        
    configureBindings();
    
}
    

    public final static CommandXboxController driverPad = new CommandXboxController(0);








    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);





    public final Swerve drivetrain = TunerConstants.createDrivetrain();
  


    /* Path follower */
    // private final SendableChooser<Command> autoChooser;



   

    private void configureBindings() {






    
        // Drivetrain
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

//Driver pad 


        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-driverPad.interpolatedLeftYAxis() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-driverPad.interpolatedLeftXAxis() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(driverPad.interpolatedRightXAxis() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

      

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}