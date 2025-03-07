package frc.robot;

import frc.robot.BulukLib.MotionControllers.Gains.Gains;

public class Constants {

  public static class controlConstants {
    public static final int DriverControllerPort = 0;
    public static final int MechanismsControllerPort = 1;
    public static final int LogitecButtonA = 1;
    public static final int LogitecButtonB = 2;  

  }

  public static class OperatorConstants {
   
    public static final double MaxElevatorSpeed = 0.70;
    public static final double MaxArmSpeed = 0.45;

    public static final double MaxElevatorPosition = -180;
    public static final double MinElevatorPosition = 0;

    public static final double DesiredMaxElevatorPosition = -180;
    public static final double DesiredMinElevatorPosition = 0;

    public static final double MaxArmPosition = 82;
    public static final double MinArmPosition = 5;

    public static final double DesiredPonerCoral = 72;
    public static final double DesiredComer = 80;
    public static final double DesiredPonerArriba = 48;
  }

  public class DriveConstants {

    public static final Gains driveGains = new Gains(0.015, 0, 0, 0.05, 0.06); //0.13
    public static final Gains turnGains = new Gains(5.0, 0, 0);
  
    public static final class frontLeft{
        public static final int DrivePort = 15; 
        public static final int TurnPort = 14; 
        public static final int EncPort = 10;
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true;
    }

    public static final class frontRight{
        public static final int DrivePort = 21; 
        public static final int TurnPort = 20; 
        public static final int EncPort = 11;  
        public static final boolean DrivemotorReversed = false;
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
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = true;
    }
  }

  public static class SensorConstants {
    public static final int MAGNETICSENSOR_DIGITAL_INPUT_PORT = 3;
    public static final int MECHANICALSWITCH_DIGITAL_INPUT_PORT = 0;
  }

  public static class idMotorConstants {
    public static final int ID_Elevator = 1;
    public static final int ID_Hanger = 2;

    public static final int ID_ArmMotor = 3;
    public static final int ID_Intake = 4;

    public static final int ID_Marco = 5;
    public static final int ID_BallIntake = 6;
    

  }


}
