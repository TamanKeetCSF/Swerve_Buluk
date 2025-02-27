package frc.robot.BulukLib.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BulukXboxController extends CommandXboxController{

    private double RIGHT_DEADBAND = 0.1;
    private double LEFT_DEADBAND = 0.1;

    /**
     * Creates a new Command Xbox Controller
     * @param USBport 
     * @param RIGHT_DEADBAND default: 0.1
     * @param LEFT_DEADBAND default: 0.1
     */
    public BulukXboxController(int USBport, double RIGHT_DEADBAND, double LEFT_DEADBAND){
        super(USBport);
        this.RIGHT_DEADBAND = RIGHT_DEADBAND;
        this.LEFT_DEADBAND = LEFT_DEADBAND;
    }

    /**
     * Creates a new Command Xbox Controller
     * @param USBport
     */
    public BulukXboxController(int USBport){
        super(USBport);
        
    }

    /**
     * Sets the sticks deadband
     * @param right new deadband (default: 0.1)
     * @param left new deadband (default: 0.1)
     */
    public void setDeadbands(double right, double left){
        this.RIGHT_DEADBAND = right;
        this.LEFT_DEADBAND = left;
    }

    /**
     * Makes the controller rumble
     * @param intensity rumble intensity from 0 to 1
     */
    public void rumble(double intensity){
        getHID().setRumble(RumbleType.kBothRumble, intensity);
    }

    /**
     * Gets the controller leftY axis
     * @return axis
     */
    public DoubleSupplier leftY(boolean inverted){
        double output = inverted ? MathUtil.applyDeadband(-getLeftY(), LEFT_DEADBAND) : MathUtil.applyDeadband(getLeftY(), LEFT_DEADBAND);
        return ()-> output;
    }

    /**
     * Gets the controller leftX axis
     * @return axis
     */
    public DoubleSupplier leftX(boolean inverted){
        double output = inverted ? MathUtil.applyDeadband(-getLeftX(), LEFT_DEADBAND) : MathUtil.applyDeadband(getLeftX(), LEFT_DEADBAND);
        return ()-> output;
    }

    /**
     * Gets the controller rightY axis
     * @return axis
     */
    public DoubleSupplier rightY(boolean inverted){
        double output = inverted ? MathUtil.applyDeadband(-getRightY(), RIGHT_DEADBAND) : MathUtil.applyDeadband(getRightY(), RIGHT_DEADBAND);
        return ()-> output;
    }

    /**
     * Gets the controller rightX axis
     * @return axis
     */
    public DoubleSupplier rightX(boolean inverted){
        double output = inverted ? MathUtil.applyDeadband(-getRightX(), RIGHT_DEADBAND) : MathUtil.applyDeadband(getRightX(), RIGHT_DEADBAND);
        return ()-> output;
    }

    /**
     * Gets the right trigger axis
     * @return axis
     */
    public DoubleSupplier RTaxis(){
        return ()-> getRightTriggerAxis();
    }

    /**
     * Gets the left trigger axis
     * @return axis
     */
    public DoubleSupplier LTaxis(){
        return ()-> getLeftTriggerAxis();
    }

    public Trigger aButton(){
        return a();
    }

    public Trigger bButton(){
        return b();
    }

    public Trigger xButton(){
        return x();
    }

    public Trigger yButton(){
        return y();
    }

    /**
     * Constructs a Trigger instance around the start button's digital signal.
     * @return Trigger instance representing the start button's digital signal attached to the default scheduler button loop.
     */
    public Trigger menu(){
        return start();
    }

    /**
     * Constructs a Trigger instance around the back button's digital signal.
     * @return Trigger instance representing the back button's digital signal attached to the default scheduler button loop.
     */
    public Trigger view(){
        return back();
    }

    /**
     * Constructs a Trigger instance based around the povDown
     * @return a Trigger instance based around the 180 degree angle of a POV on the HID.
     */
    public Trigger dPadDown(){
        return povDown();
    }

    /**
     * Constructs a Trigger instance based around the povUP
     * @return a Trigger instance based around the 180 degree angle of a POV on the HID.
     */
    public Trigger dPadUp(){
        return povUp();
    }

    /**
     * Constructs a Trigger instance based around the povLeft
     * @return a Trigger instance based around the 180 degree angle of a POV on the HID.
     */
    public Trigger dPadLeft(){
        return povLeft();
    }

    /**
     * Constructs a Trigger instance based around the povRight
     * @return a Trigger instance based around the 180 degree angle of a POV on the HID.
     */
    public Trigger dPadRight(){
        return povRight();
    }


    /**
     * Constructs a Trigger instance based around the left trigger with custom threshold
     * @return a Trigger instance based around the 180 degree angle of a POV on the HID.
     */
    public Trigger LT(double threshold){
        return leftTrigger(threshold);
    }

     /**
     * Constructs a Trigger instance based around the left trigger with default threshold
     * @return a Trigger instance based around the 180 degree angle of a POV on the HID.
     */
    public Trigger LT(){
        return leftTrigger();
    }

     /**
     * Constructs a Trigger instance based around the right trigger with custom threshold
     * @return a Trigger instance based around the 180 degree angle of a POV on the HID.
     */
    public Trigger RT(double threshold){
        return rightTrigger(threshold);
    }

    /**
     * Constructs a Trigger instance based around the right trigger with default threshold
     * @return a Trigger instance based around the 180 degree angle of a POV on the HID.
     */
    public Trigger RT(){
        return rightTrigger();
    }

    /**
     * Constructs a Trigger instance around the right stick button's digital signal.
     * @return a Trigger instance representing the right stick button's digital signal attached to the default scheduler button loop.
     */
    public Trigger R3(){
        return rightStick();
    }

    /**
     * Constructs a Trigger instance around the left stick button's digital signal.
     * @return a Trigger instance representing the left stick button's digital signal attached to the default scheduler button loop.
     */
    public Trigger L3(){
        return leftStick();
    }

    /**
     * Constructs a Trigger instance around the right bumper button's digital signal.
     * @return a Trigger instance representing the right bumper button's digital signal attached to the default scheduler button loop.
     */
    public Trigger RB(){
        return rightBumper();
    }

    /**
     * Constructs a Trigger instance around the left bumper button's digital signal.
     * @return a Trigger instance representing the left bumper button's digital signal attached to the default scheduler button loop.
     */
    public Trigger LB(){
        return leftBumper();
    }

    /** ;v
     * Creates a trigger combo 
     * @param first Trigger
     * @param second Trigger
     * @return the combo (first + second)
     */
    public Trigger combo(Trigger first, Trigger second){
        return first.and(second);
    }

    /**
     * Creates a or Trigger
     * @param first Trigger
     * @param second Trigger
     * @return the or Trigger
     */
    public Trigger or(Trigger first, Trigger second){
        return first.or(second);
    }


    
}
