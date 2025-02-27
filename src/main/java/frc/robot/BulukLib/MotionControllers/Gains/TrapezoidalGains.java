package frc.robot.BulukLib.MotionControllers.Gains;

public class TrapezoidalGains {
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double acc;
    public double velocity;

    public TrapezoidalGains(double kP, double kI, double kD, double kS, double kV, double acc, double velocity){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.acc = acc;
        this.velocity = velocity;
    }

    public void setKp(double kP){
        this.kP = kP;
    }

    public void setKi(double kI){
        this.kI = kI;
    }

    public void setKd(double kD){
        this.kD = kD;
    }

    public void setKs(double kS){
        this.kS = kS;
    }

    public void setKv(double kV){
        this.kV = kV;
    }

    public void setAcc(double acc){
        this.acc = acc;
    }

    public void setVelocity(double velocity){
        this.velocity = velocity;
    }

    public double[] toArray(){
        return new double[]{kP, kI, kD, kS, kV, acc, velocity};
    }

    public TrapezoidalGains toGains(double[] gains){
        return new TrapezoidalGains(gains[0], gains[1], gains[2], gains[3], gains[4], gains[5], gains[6]);
    }


    
}