package frc.robot.BulukLib.MotionControllers.TrapezoidalControl;

public class TrapezoidalTolerance {
    private double positionTolerance;
    private double velocityTolerance;

    public TrapezoidalTolerance(double positionTolerance, double velocityTolerance) {
        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;
    }

    public double getPositionTolerance() {
        return positionTolerance;
    }

    public double getVelocityTolerance() {
        return velocityTolerance;
    }
    
}