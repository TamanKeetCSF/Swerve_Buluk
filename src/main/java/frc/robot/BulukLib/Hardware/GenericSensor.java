package frc.robot.BulukLib.Hardware;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GenericSensor{

    public enum TriggerValue{
        kDefault, kInvert
    }

    private DigitalInput beam;

    private TriggerValue triggerOutput;

    private Debouncer delayer;

    public GenericSensor(int port, TriggerValue triggerOutput){
        this.triggerOutput = triggerOutput;
        this.beam = new DigitalInput(port);
    }

    private boolean shouldInvert(){
        if (triggerOutput == TriggerValue.kDefault) {
            return false;
        }
        else{
            return true;
        }
    }

    public boolean get(){
        return shouldInvert() ? !beam.get(): beam.get();
    }

    public BooleanSupplier getSupplier(){
        return ()-> get();
    }

    public Trigger getTrigger(){
        return new Trigger(getSupplier());
    }

    public boolean getDelayed(double delay){
        this.delayer = new Debouncer(delay);
        boolean output = shouldInvert() ? !beam.get(): beam.get();
        return delayer.calculate(output);
    }

    public BooleanSupplier getDelayedSupplier(double delay){
        this.delayer = new Debouncer(delay);
        boolean output = shouldInvert() ? !beam.get(): beam.get();
        return ()-> delayer.calculate(output);
    }

    public Trigger getTriggerDelayed(double delay){
        this.delayer = new Debouncer(delay);
        boolean output = shouldInvert() ? !beam.get(): beam.get();
        return new Trigger(()-> delayer.calculate(output));
    }
}
