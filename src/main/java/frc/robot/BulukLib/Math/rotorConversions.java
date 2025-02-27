package frc.robot.BulukLib.Math;

public class rotorConversions {

    //Evita instanciarlo afuera
    private rotorConversions(){}

    public static double rotorRotationsToMeters(double rotations, double reduction, double WheelRadiusInMeters){
        double positionFactor = 2 * Math.PI / reduction;
        double convertion = rotations * positionFactor; 
        return convertion * WheelRadiusInMeters;
    }

    public static double rotorRPMtoMetersPerSec(double RPM, double reduction, double WheelRadiusInMeters){
        double velocityFactor = (2 * Math.PI) / 60 / reduction;

        double convertion = RPM * velocityFactor;

        return convertion * WheelRadiusInMeters;
    }

    public static double rotorRotationsToWheelRadians(double rotations, double reduction){
        double positionFactor = 2 * Math.PI / reduction;
        return rotations * positionFactor;
    }

    public static double rotorRotationsToWheelRadPerSec(double RPM, double reduction){
        double velocityFactor = (2 * Math.PI) / 60 / reduction;

        return RPM * velocityFactor;
    }

    public static double metersPerSecToWheelRadPerSec(double mps,double WheelRadiusInMeters){
        return mps / WheelRadiusInMeters;
    }

    public static double metersToWheelRadians(double meters, double WheelRadiusInMeters){
        return meters / WheelRadiusInMeters;
    }
}
