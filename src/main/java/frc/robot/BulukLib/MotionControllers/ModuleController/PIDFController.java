package frc.robot.BulukLib.MotionControllers.ModuleController;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl.OutputType;
import frc.robot.BulukLib.MotionControllers.Gains.FeedForwardGains;
import frc.robot.BulukLib.MotionControllers.Gains.PIDGains;

public class PIDFController{

    private FeedForwardGains gainsFF;
    private PIDController controller;
    private OutputType outputType;
    private double output;

    public PIDFController(PIDGains gainsPID, FeedForwardGains gainsFF, OutputType outputType){
        this.gainsFF = gainsFF;
        this.outputType = outputType;
        this.output = 0;
        this.controller = new PIDController(gainsPID.kp, gainsPID.ki, gainsPID.kd);

    }

    public double calculate(double currentVelocity, double goalVelocity){

        double ffVolts = gainsFF.ks * Math.signum(goalVelocity) + gainsFF.kv * goalVelocity;

        double pidOutput = controller.calculate(currentVelocity, goalVelocity);

        this.output = pidOutput + ffVolts;

        if (outputType == OutputType.kNegative) {
            return -this.output;
        }else{
            return this.output;
        }
    }

    public double getAppliedOutput(){
        return output;
    }

}
