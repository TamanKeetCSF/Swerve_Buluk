package frc.robot.BulukLib.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class QoLUtil {
    //metodos para hacer el código más flexible y menos tedioso de manejar

    public static double square(double x){
        return x * x;
    }
    public static double invertIf(double x, boolean v){
        return v? -x: x;
    }
    public static double percentageOf(double percentage, double x){
        double y = percentage * 0.01;
        return x * y;
    }

    public static Rotation2d negativeOf(Rotation2d rotation2d){
        return new Rotation2d(-rotation2d.getRadians());
    }

    public static Translation2d negativeOf(Translation2d translation){
        return new Translation2d(-translation.getX(), -translation.getY());
    }

}
