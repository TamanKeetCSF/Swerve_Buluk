package frc.robot.BulukLib.Swerve;

import edu.wpi.first.math.util.Units;

public class SwerveConfig{

    public class speeds {
        public static final double MAX_LINEAR_SPEED = Units.feetToMeters(19.0); //5.7912
        public static final double TRUE_MAX_LINEAR_SPEED = 5.6;
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / measures.DRIVE_BASE_RADIUS;
        public static final double TRUE_MAX_ANGULAR_SPEED = TRUE_MAX_LINEAR_SPEED / measures.DRIVE_BASE_RADIUS;
        public static final double MAX_ROTATION_VEL = Units.rotationsToRadians(10);
    }
    
    public class measures{
        public static final double WHEELRADIUS = Units.inchesToMeters(2.0);
        public static final double WHEELDIAMETER = Units.inchesToMeters(4.0);
        public static final double TRACK_WIDTH_X = 0.30; 
        public static final double TRACK_WIDTH_Y = 0.305;
        public static final double robotMassKg = 57.45; 
        public static final double robotMOI = 5.16;  
        public static final double wheelCOF = 1.0;

        public static final double DRIVE_BASE_RADIUS =
            Math.hypot(TRACK_WIDTH_X, TRACK_WIDTH_Y);
           
    }

    public class currentLimiting{
        public static final int driveCurrentLimit = 40; 
        public static final int turnCurrentLimit = 20;
        
    }

    public class reductions{
        public static final double DriveReduction = 7.375;
        public static final double TurnReduction = 18.75;  
    }

}
