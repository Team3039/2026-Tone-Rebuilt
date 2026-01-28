package frc.robot;



public class Constants {

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
        public static final double FLYWHEEL_KP = 0.009;
        public static final double FLYWHEEL_KI = 0;
        public static final double FLYWHEEL_KD = 0.0000;
        public static final double FLYWHEEL_KS = 0.0001;
        public static final double FLYWHEEL_KG = 0;
        public static final double FLYWHEEL_KV = 0;
        public static final double FLYWHEEL_MAX_VEL = .0;
        public static final double FLYWHEEL_MAX_ACCEL = .00;
        public static final double FLYWHEEL_OFFSET = 0;

    }
       

    }


