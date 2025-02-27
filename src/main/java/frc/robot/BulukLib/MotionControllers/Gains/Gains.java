//v1.0.2
package frc.robot.BulukLib.MotionControllers.Gains;

/**
 * Represents PID and motion parameters for control systems.
 */
public class Gains {
    
    private final double kP,kI,kD,kS,kV, acc, jerk;

    public Gains(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = 0;
        this.kS = 0;
        this.acc = 0;
        this.jerk = 0;
    }

    public Gains(double kP, double kI, double kD, double kS, double kV){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kS = kS;
        this.acc = 0;
        this.jerk = 0;
    }

    public Gains(double kP, double kI, double kD, double kS, double kV, double acc, double jerk){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kS = kS;
        this.acc = acc;
        this.jerk = jerk;
    }
    
    //Getters
    public double getAcceleration(){
        return acc;
    }
    public double getJerk(){
        return jerk;
    }
    public double getP(){
        return kP;
    }
    public double getI(){
        return kI;
    }
    public double getD(){
        return kD;
    }
    public double getS(){
        return kS;
    }
    public double getV(){
        return kV;
    }

    public double[] getPID(){
        return new double[]{kP,kI,kD};
    }

    public Gains toPID(double[] pid){
        return new Gains(pid[0], pid[1], pid[2]);
    }

}
