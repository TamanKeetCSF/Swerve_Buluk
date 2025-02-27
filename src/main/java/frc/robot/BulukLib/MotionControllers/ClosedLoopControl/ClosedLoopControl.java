package frc.robot.BulukLib.MotionControllers.ClosedLoopControl;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BulukLib.Dashboard.Elastic;
import frc.robot.BulukLib.Dashboard.Elastic.Notification;
import frc.robot.BulukLib.Dashboard.Elastic.Notification.NotificationLevel;
import frc.robot.BulukLib.MotionControllers.Gains.Gains;

public class ClosedLoopControl {

    public enum OutputType{
        kPositive, kNegative
    }

    private Gains gains;
    private Gains defaultGains;
    private OutputType output;
    private PIDController controller;
    private double outputCL = 0;
    private double reference = 0;
    private String keyTune = "";
    private double tolerance = 0;
    private double DEADBAND = 0.01;
    private boolean feedForwardEnabled = false;
    private boolean resetError = false;

    private Notification notification = new Notification(NotificationLevel.WARNING, "ClosedLoop", "Updating gains!");
    private Notification notification2 = new Notification(NotificationLevel.WARNING, "ClosedLoop", "Updating tolerance!");
    private Notification notification3 = new Notification(NotificationLevel.WARNING, "ClosedLoop", "Updating output!");

    private SendableChooser<OutputType> typeC;

    private double startTime = 0;
    private boolean timingActive = false;
    private double lastSetpoint = 0;
    private double elapsedTime = 0;
    private SimpleMotorFeedforward ff;

    public ClosedLoopControl(Gains gains, OutputType output){

        this.gains = gains;
        this.defaultGains = new Gains(gains.getP(), gains.getI(), gains.getD());
        this.output = output;

        this.controller = new PIDController(gains.getP(), gains.getI(), gains.getD());
    }

    public void enableContinuousInput(double x){
        controller.enableContinuousInput(-x, x);
    }

    public void LogTimeToSetpoint() {
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

    public double getCurrentError(){
        return controller.getError();
    }


    public void setOutputType(OutputType type){
        Elastic.sendNotification(notification3);
        this.output = type;
    }

    public void disableContinuousInput(){
        controller.disableContinuousInput();
    }

    public void enableContinuousInput(double x, double y){
        controller.enableContinuousInput(x, y);
    }

    public Gains currentGains(){
        return gains;
    }

    public void setTolerance(double x){
        if (this.tolerance != x) {
            this.tolerance = x;
            controller.setTolerance(x);
            Elastic.sendNotification(notification2);
        }
    }

    public double getTolerance(){
        return tolerance;
    }

    public void updateSetpoint(double newSetpoint) {
        if (newSetpoint != lastSetpoint) {
            lastSetpoint = newSetpoint;
            startTime = System.nanoTime() / 1e9; //Store current time in seconds
            timingActive = true;
        }
    }
    
    public void setGains(Gains newGains){

        if (!this.gains.equals(newGains)) {
            this.gains = newGains;
            controller.setPID(newGains.getP(), newGains.getI(), newGains.getD());
            Elastic.sendNotification(notification);
        }
    }

    public void reset(){
        controller.reset();
    }

    public boolean atGoal(){
        return controller.atSetpoint();
    }

    public OutputType getOutputType(){
        return output;
    }

    public double getSetpoint(){
        return controller.getSetpoint();
    }

    public double getOutput(){
        return outputCL;
    }

    public double runRequest(ClosedLoopRequest mRequest){
        return mRequest.asDouble();
    }
    public double getReference(){
        return reference;
    }

    public boolean isFeedforwardEnabled(){
        return feedForwardEnabled;
    }

    private void setM(double x){
        this.reference = x;
    }

    private void setF(boolean f){
        this.feedForwardEnabled = f;
    }

    public boolean ErrorResetAtSetpoint(){
        return resetError;
    }

    public void resetAtSetpoint(boolean v){
        this.resetError = v;
    }

    public void graph(String key){

        //Logs the time to setpoint
        LogTimeToSetpoint();

        SmartDashboard.putNumber("["+ key +"]" + " Tolerance:", getTolerance());
        SmartDashboard.putNumber("["+ key +"]" + " Output:", outputCL);
        SmartDashboard.putString("["+ key +"]" + " OutputType:", output.toString());
        SmartDashboard.putNumber("["+ key +"]" +" Setpoint:", controller.getSetpoint());
        SmartDashboard.putNumber("["+ key +"]" +" Last Known Setpoint:", lastSetpoint);
        SmartDashboard.putNumber("["+ key +"]" +" Error:", controller.getError());
        SmartDashboard.putBoolean("["+ key +"]" +" AtGoal:", controller.atSetpoint());
        SmartDashboard.putBoolean("["+ key +"]" +" ResetingOnGoal:", ErrorResetAtSetpoint());
        SmartDashboard.putBoolean("["+ key +"]" +" withFeedforward:", isFeedforwardEnabled());
        SmartDashboard.putNumber("["+ key +"]" +" Reference:", getReference());
        SmartDashboard.putNumberArray("["+ key +"]" +"PIDGains:", gains.getPID());
        SmartDashboard.putNumberArray("["+ key +"]" +"FFGains:", new Double[]{gains.getS(), gains.getV()});
        SmartDashboard.putNumber("["+  key +"]"+ "TimeToSetpoint:", getTimeToSetpoint());
        
    }

    public void initTuning(String key){

        this.keyTune = key;

        typeC = new SendableChooser<>();

        typeC.setDefaultOption("kPositive", OutputType.kPositive);
        typeC.addOption("kNegative", OutputType.kNegative);

        notification.setTitle("[" + keyTune +"]" + "GAINS MODIFIED!");
        notification2.setTitle("[" + keyTune +"]" + "TOLERANCE CHANGED!");
        notification3.setTitle("[" + keyTune +"]" + "OUTPUT CHANGED!");

        SmartDashboard.putNumber("[" + key + "] P: ", gains.getP());
        SmartDashboard.putNumber("[" + key + "] I: ", gains.getI());
        SmartDashboard.putNumber("[" + key + "] D: ", gains.getD());
        SmartDashboard.putNumber("[" + key + "] Tolerance: ", getTolerance());
        SmartDashboard.putData("[" + key + "] OutputType: ", typeC);
        SmartDashboard.putBoolean("[" + key + "] RESET: ", false);

    }

    public void tuneWithInterface(){

        double[]pid = new double[3];

        pid[0] = SmartDashboard.getNumber("[" + keyTune + "] P: ", gains.getP());
        pid[1] = SmartDashboard.getNumber("[" + keyTune + "] I: ", gains.getI());
        pid[2] = SmartDashboard.getNumber("[" + keyTune + "] D: ", gains.getD());

        boolean reset = SmartDashboard.getBoolean("[" + keyTune + "] RESET: ", false);

        double newTolerance = SmartDashboard.getNumber("[" + keyTune + "] Tolerance: ", getTolerance());

        OutputType newOutput = typeC.getSelected();

        //Update the values
        
        if (newTolerance != getTolerance()) {
            setTolerance(newTolerance);
        }

        if (newOutput != output) {
            setOutputType(newOutput);
        }

        // Reset to default gains if button is pressed
        if (reset) {
            setGains(defaultGains);

            setTolerance(0);

            // **Update SmartDashboard values to reflect default gains**
            SmartDashboard.putNumber("[" + keyTune + "] P: ", defaultGains.getP());
            SmartDashboard.putNumber("[" + keyTune + "] I: ", defaultGains.getI());
            SmartDashboard.putNumber("[" + keyTune + "] D: ", defaultGains.getD());
            SmartDashboard.putNumber("[" + keyTune + "] Tolerance: ", getTolerance());
        
            // Reset the button to false so it doesn't continuously reset
            SmartDashboard.putBoolean("[" + keyTune + "] RESET: ", false);
        
        } else {
            // Update gains if they changed
            if (pid.length == 3 && (pid[0] != gains.getP() || pid[1] != gains.getI() || pid[2] != gains.getD())) {
            setGains(new Gains(pid[0], pid[1], pid[2], gains.getS(), gains.getV()));
            }
        }      
    }

    public class ClosedLoopRequest {

        public double reference;
        private boolean withFeedForward = false;
        public double setpoint = 0;
        private boolean reset = false;
        private boolean clampOutput = false;
        private double cap = 0;

        public ClosedLoopRequest(){
            this.reference = 0;
            this.setpoint = 0;
            updateSetpoint(0);

        }

        public ClosedLoopRequest withReference(double Reference){
            this.reference = Reference;
            setM(Reference);
            return this;
        }

        public ClosedLoopRequest toSetpoint(double setpoint){
            this.setpoint = setpoint;
            updateSetpoint(setpoint);
            return this;
        }

        public ClosedLoopRequest toZero(){
            this.setpoint = 0;
            updateSetpoint(0);
            return this;
        }

        public ClosedLoopRequest withFeedForward(boolean enable){
            this.withFeedForward = enable;
            if (enable) {
                ff = new SimpleMotorFeedforward(gains.getS(), gains.getV());
            }

            setF(enable);
            return this;
        }

        public ClosedLoopRequest withTolerance(double tolerance){
            setTolerance(tolerance);
            return this;
        }

        public ClosedLoopRequest withGains(Gains newG){
            setGains(newG);
            return this;
        }

        public ClosedLoopRequest withContinuousInput(double x, double y){
            enableContinuousInput(x, y);
            return this;
        }

        public ClosedLoopRequest withContinuousInput(double x){
            enableContinuousInput(-x, x);
            return this;
        }

        public ClosedLoopRequest withOutputType(OutputType type){
            setOutputType(type);
            return this;
        }

        public ClosedLoopRequest resetOnSetpoint(boolean enable){
            this.reset = enable;
            resetError = enable;
            return this;
        }

        public ClosedLoopRequest enableOutputClamp(boolean enable){
            this.clampOutput = enable;
            return this;
        }

        public ClosedLoopRequest withClamp(double cap){
            this.cap = cap;
            return this;
        }

        public boolean isFeedForwardEnabled(){
            return withFeedForward;
        }

        public double asDouble(){


            double output = controller.calculate(reference, setpoint);
            
            if (withFeedForward) {
                output += ff.calculate(setpoint);
            }

            if (reset && atGoal()) {
                controller.reset();
                reset = false;            
            }

            if (Math.abs(output) < DEADBAND) {
                output = 0;
            }

            if (getOutputType() == OutputType.kNegative) {
                output = -output;
            }

            if (clampOutput) {
                Math.min(Math.max(output, -cap), cap);
            }

            outputCL = output;
            return output;
        }

    }

}
