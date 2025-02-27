package frc.robot.BulukLib.MotionControllers.Gains;

public class PIDGains {
    public double kp;
    public double ki;
    public double kd;

    public PIDGains(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
}
