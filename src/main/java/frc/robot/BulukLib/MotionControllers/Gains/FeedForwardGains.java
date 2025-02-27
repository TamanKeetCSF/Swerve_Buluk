package frc.robot.BulukLib.MotionControllers.Gains;

public class FeedForwardGains {

    public double ks;
    public double kv;
    public double ka;

    public FeedForwardGains(double ks, double kv, double ka){
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
    }
}
