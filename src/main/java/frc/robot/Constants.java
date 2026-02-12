package frc.robot;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class Constants {

    private static final double wheelBase = 0.5; // Define the wheelBase (example value)
    private static final double trackWidth = 0.5; // Define the trackWidth (example value)
    public static final double kDriveGearRatio = 5.357142857142857;
    public static final double wheelCircumference = Units.inchesToMeters(4 * Math.PI);



  public static final APConstraints kConstraints = new APConstraints()
            .withAcceleration(5.0)
            .withJerk(5);

    public static final APProfile kProfile = new APProfile(kConstraints)
            .withErrorXY(Centimeters.of(1.5))
            .withErrorTheta(Degrees.of(3))
            .withBeelineRadius(Centimeters.of(8));

    public static final Autopilot kAutopilot = new Autopilot(kProfile);



public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));


    public static final class Ports {
//CLAW
    public static final int INTAKE = 1;
    public static final int INDEXER = 2;
    public static final int INDEXERCANRANGE = 3;
    public static final int TURRET = 4;
    public static final int FLYWHEEL_LEFT = 5;
    public static final int FLYWHEEL_RIGHT = 6;

//TURRET
//INDEXERCANRANGE
    //dio ports
    public static final int INTAKE_ENCODER = 9;





    }

    public static final class Intake {
        public static final double Intake_KP = 0.009;
        public static final double Intake_KI = 0;
        public static final double Intake_KD = 0.0000;
        public static final double Intake_KS = 0.0001;
        public static final double Intake_KG = 0;
        public static final double Intake_KV = 0;
        public static final double Intake_MAX_VEL = .0;
        public static final double Intake_MAX_ACCEL = .00;
        public static final double Intake_OFFSET = 0;

    }

     public static final class Turret {
        public static final double Turret_KP = 0.009;
        public static final double Turret_KI = 0;
        public static final double Turret_KD = 0.0000;
        public static final double Turret_KS = 0.0001;
        public static final double Turret_KG = 0;
        public static final double Turret_KV = 0;
        public static final double Turret_MAX_VEL = .0;
        public static final double Turret_MAX_ACCEL = .00;
        public static final double Turret_OFFSET = 0;
        

    }
     public static final class Flywheel {
      public static final double Flywheel_KP = 0.12;
		public static final double Flywheel_KI = 0.000;
		public static final double Flywheel_KD = 0.000;
        public static final double Flywheel_FF = 1.75;


    }
       

    }


