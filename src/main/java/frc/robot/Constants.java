package frc.robot;

import frc.robot.BulukLib.MotionControllers.Gains.Gains;

public class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double MaxElevatorSpeed = 0.4;
    public static final double MaxArmSpeed = 0.07;

    public static final double MaxElevatorPosition = -100;
    public static final double MinElevatorPosition = -4;

    public static final double DesiredMaxElevatorPosition = -100;
    public static final double DesiredMinElevatorPosition = -4;

    public static final double MaxArmPosition = 70;
    public static final double MinArmPosition = 5;

    public static final double DesiredMaxArmPosition = 30;
    public static final double DesiredMinArmPosition = 5;
  }

  public class DriveConstants {

    public static final Gains driveGains = new Gains(0.015, 0, 0, 0.05, 0.06); //0.13
    public static final Gains turnGains = new Gains(5.0, 0, 0);
  
    public static final class frontLeft{

        public static final int DrivePort = 15; 
        public static final int TurnPort = 14; 
        public static final int EncPort = 10;                                                                                                                                                                                                                                                                                                                                                                                                                                                    ; //48     //93  //138      //48 o 138 o 228
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true;

    }

    public static final class frontRight{

        public static final int DrivePort = 21; 
        public static final int TurnPort = 20; 
        public static final int EncPort = 11;  
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true;

    }

    public static final class backLeft{

        public static final int DrivePort = 17; 
        public static final int TurnPort = 16; 
        public static final int EncPort = 13;  
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true; 

    }

    public static final class backRight{

        public static final int DrivePort = 23; 
        public static final int TurnPort = 22; 
        public static final int EncPort = 12; 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true;

    }
  }

  public static class SensorConstants {
    public static final int MAGNETICSENSOR_DIGITAL_INPUT_PORT = 2;
    public static final int MECHANICALSWITCH_DIGITAL_INPUT_PORT = 0;
  }

}
