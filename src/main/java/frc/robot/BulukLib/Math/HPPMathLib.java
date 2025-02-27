package frc.robot.BulukLib.Math;

import java.lang.Math;

import edu.wpi.first.math.util.Units;

public class HPPMathLib {

    public static double coterminalgrados(double angle){
        if (angle >= 0 && angle < 360) {
            return angle;
        }
        else if (angle > 360) {
            return angle - Math.floor(angle / 360) * 360;
        }
        else {
            return angle + (1 + Math.floor(-angle / 360)) * 360;
        }
    }
    public static double coterminalradianes(double angle){
        if (angle >= 0 && angle < 2 * Math.PI) {
            return angle;
        }
        else if (angle >  2 * Math.PI) {
            return angle - Math.floor(angle /  (2 * Math.PI)) *  (2 * Math.PI);
        }
        else {
            return angle + (1 + Math.floor(-angle /  (2 * Math.PI))) *  (2 * Math.PI);
        }
    }

    public static double MinAngleDeg(double ang_from, double ang_to) {
        if (Math.abs(ang_to - ang_from) < coterminalgrados(ang_to - ang_from)) {
            return ang_to - ang_from;
        }
        else {
            return coterminalgrados(ang_to - ang_from);
        }
    }
    public static double MinAngleRad(double ang_from, double ang_to) {
        if (Math.abs(ang_to - ang_from) < coterminalradianes(ang_to - ang_from)) {
            return ang_to - ang_from;
        }
        else {
            return coterminalradianes(ang_to - ang_from);
        }
    }

    public static double atan3(double x, double y, double joystickThreshold, double defaultValue) {
        if (Math.abs(x * x + y * y) < joystickThreshold) {
            return defaultValue;
        }

        return Units.radiansToDegrees(Math.atan2(y, x));

    }
    
    
}

