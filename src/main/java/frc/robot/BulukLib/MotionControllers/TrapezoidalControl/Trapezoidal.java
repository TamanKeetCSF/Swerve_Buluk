package frc.robot.BulukLib.MotionControllers.TrapezoidalControl;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl.OutputType;
import frc.robot.BulukLib.MotionControllers.Gains.TrapezoidalGains;

public class Trapezoidal {
    private TrapezoidalGains gains;
    private final double period = 0.02;
    private final TrapezoidProfile.Constraints constraints;
    private final SimpleMotorFeedforward ff;
    private final ProfiledPIDController controller;
    private OutputType outputType;
    private double startTime = 0;
    private boolean timingActive = false;
    private double elapsedTime = 0;
    private double output;
    private double measurement;
    private TrapezoidalTolerance tolerance;
    private String keyTune;
    private SendableChooser<OutputType> chooser;

    public Trapezoidal(TrapezoidalGains gains, OutputType outputType) {
        this.gains = gains;
        this.measurement = 0;
        this.tolerance = new TrapezoidalTolerance(0.1, 0.1);
        this.outputType = outputType;
        output = 0;
        this.keyTune = "";

        constraints = new TrapezoidProfile.Constraints(gains.velocity, gains.acc);

        ff = new SimpleMotorFeedforward(gains.kS, gains.kV);

        controller = new ProfiledPIDController(
            gains.kP,
            gains.kI,
            gains.kD,
            constraints,
            period
        );
    }

    public ProfiledPIDController get(){
        return controller;
    }

    public void setGains(TrapezoidalGains gains) {
        this.gains = gains;
        controller.setPID(gains.kP, gains.kI, gains.kD);
        controller.setConstraints(new TrapezoidProfile.Constraints(gains.velocity, gains.acc));
    }

    public void setTolerance(TrapezoidalTolerance tolerance) {
        this.tolerance = tolerance;
        controller.setTolerance(tolerance.getPositionTolerance(), tolerance.getVelocityTolerance());
    }

    public double run(double measurement, State goal) {
        this.measurement = measurement;
        double feedforwardOutput = ff.calculate(controller.getSetpoint().velocity) / RobotController.getBatteryVoltage();
        this.output = controller.calculate(measurement, goal) + feedforwardOutput;

        if (outputType == OutputType.kNegative) {
            return -output;
        } else {
            return output;
            
        }
    }

    public double run(double measurement, double goal) {
        this.measurement = measurement;
        double feedforwardOutput = ff.calculate(controller.getSetpoint().velocity) / RobotController.getBatteryVoltage();
        this.output = controller.calculate(measurement, goal) + feedforwardOutput;

        if (outputType == OutputType.kNegative) {
            return -output;
        } else {
            return output;
            
        }
    }

    public double run(double measurement){
        this.measurement = measurement;
        double feedforwardOutput = ff.calculate(controller.getSetpoint().velocity) / RobotController.getBatteryVoltage();
        this.output = controller.calculate(measurement) + feedforwardOutput;

        if (outputType == OutputType.kNegative) {
            return -output;
        } else {
            return output;
            
        }

    }

    public TrapezoidalGains getGains() {
        return gains;
    }

    public void reset(double measurement){
        this.measurement = measurement;
        controller.reset(measurement);
    }

    public double getOutput() {
        return output;
    }

    public boolean atGoal(){
        return controller.atSetpoint();
    }

    public OutputType getOutputType(){
        return outputType;
    }

    public State getSetpoint(){
        
        return controller.getSetpoint();
    }

    private void LogTimeToSetpoint() {
        if (timingActive && atGoal()) {
            this.elapsedTime = (System.nanoTime() / 1e9) - startTime;
            timingActive = false;
        }else if (!timingActive && !atGoal()) {
            // If we've left the goal, reset timing for the next approach
            timingActive = true;
            startTime = System.nanoTime() / 1e9;
        }
    }

    public double getTimeToSetpoint(){
        return elapsedTime;
    }

    public void setOutputType(OutputType type){
        this.outputType = type;
    }

    public void setGoal(double value){
        controller.setGoal(value);
    }

    public void graph(String key){

        //Logs the time to setpoint
        LogTimeToSetpoint();

        SmartDashboard.putNumberArray("["+ key +"]" + " Tolerance:", new Double[] {tolerance.getPositionTolerance(), tolerance.getVelocityTolerance()});
        SmartDashboard.putNumber("["+ key +"]" + " Output:", getOutput());
        SmartDashboard.putString("["+ key +"]" + " OutputType:", outputType.toString());
        SmartDashboard.putNumberArray("["+ key +"]" +" Setpoint:", new Double[] {controller.getSetpoint().position, controller.getSetpoint().velocity});
        SmartDashboard.putNumber("["+ key +"]" +" Error:", controller.getPositionError());
        SmartDashboard.putBoolean("["+ key +"]" +" AtGoal:", controller.atSetpoint());
        SmartDashboard.putNumber("["+ key +"]" +" Reference:", measurement);
        SmartDashboard.putNumberArray("["+ key +"]" +"TrapezoidalGains:", gains.toArray());
        SmartDashboard.putNumber("["+  key +"]"+ "TimeToSetpoint:", getTimeToSetpoint());
        
    }

    public void initTuning(String keyTune){
        this.keyTune = keyTune;

        this.chooser = new SendableChooser<>();

        chooser.setDefaultOption("kPositive", OutputType.kPositive);
        chooser.addOption("kNegative", OutputType.kNegative);

        SmartDashboard.putNumber("[" + keyTune + "] P: ", gains.kP);
        SmartDashboard.putNumber("[" + keyTune + "] I: ", gains.kI);
        SmartDashboard.putNumber("[" + keyTune + "] D: ", gains.kD);
        SmartDashboard.putNumber("[" + keyTune + "] Acceleration ", gains.acc);
        SmartDashboard.putNumber("[" + keyTune + "] Velocity ", gains.velocity);
        SmartDashboard.putNumber("[" + keyTune + "] Tolerance Position: ", tolerance.getPositionTolerance());
        SmartDashboard.putNumber("[" + keyTune + "] Tolerance Velocity: ", tolerance.getVelocityTolerance());
        SmartDashboard.putData("[" + keyTune + "] OutputType: ", chooser);

    }

    public void Tune(){

        double[]tuner = new double[5];

        tuner[0] = SmartDashboard.getNumber("[" + keyTune + "] P: ", gains.kP);
        tuner[1] = SmartDashboard.getNumber("[" + keyTune + "] I: ", gains.kI);
        tuner[2] = SmartDashboard.getNumber("[" + keyTune + "] D: ", gains.kD);
        tuner[3] = SmartDashboard.getNumber("[" + keyTune + "] Acceleration ", gains.acc);
        tuner[4] = SmartDashboard.getNumber("[" + keyTune + "] Velocity ", gains.velocity);

        double newPositionTolerance = SmartDashboard.getNumber("[" + keyTune + "] Tolerance: ", tolerance.getPositionTolerance());
        double newVelocityTolerance = SmartDashboard.getNumber("[" + keyTune + "] Tolerance: ", tolerance.getVelocityTolerance());

        OutputType newOutput = chooser.getSelected();

        if (newPositionTolerance != tolerance.getPositionTolerance()) {
            setTolerance(new TrapezoidalTolerance(newPositionTolerance, tolerance.getVelocityTolerance()));
        }

        if (newVelocityTolerance != tolerance.getVelocityTolerance()) {
            setTolerance(new TrapezoidalTolerance(tolerance.getPositionTolerance(), newVelocityTolerance));
        }

        if (newOutput != outputType) {
            setOutputType(newOutput);
        }

        if(tuner.length == 5 && (tuner[0] != gains.kP || tuner[1] != gains.kI || tuner[2] != gains.kD ||
            tuner[3] != gains.acc || tuner [4] != gains.velocity)){
            setGains(new TrapezoidalGains(tuner[0], tuner[1], tuner[2], gains.kS, gains.kV, tuner[3], tuner[4]));
        }


    }

}