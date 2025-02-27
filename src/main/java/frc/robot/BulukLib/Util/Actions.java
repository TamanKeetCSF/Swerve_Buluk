package frc.robot.BulukLib.Util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;

public class Actions{

    static boolean flag = false;
       
        public static void runOnce(BooleanSupplier condition, Runnable action){
    
        if (condition.getAsBoolean() && !flag) {
            action.run();
            flag = true;
        }else if (!condition.getAsBoolean()) {
            flag = false;
        }
    }

    public static void runWhenAll(BooleanSupplier[]conditions, Runnable action){
        boolean allTrue = true;
        for(BooleanSupplier condition:conditions){
            if (!condition.getAsBoolean()) {
                allTrue = false;
                break;
            }
        }
        if (allTrue) {
            action.run();
        }
    }

    public static void debounceAction(Debouncer timer, BooleanSupplier condition, Runnable action){

        if (timer.calculate(condition.getAsBoolean())) {
            action.run();
        }
    }

    public static void runWhenAny(BooleanSupplier[] conditions, Runnable action) {
        for (BooleanSupplier condition : conditions) {
            if (condition.getAsBoolean()) {
                action.run();
                break;
            }
        }
    }

    public static void runWithDeadline(BooleanSupplier condition, long timeoutMillis, Runnable action, Runnable onTimeout) {
        long startTime = System.currentTimeMillis();
        
            if (condition.getAsBoolean()) {
                action.run();
            } else if (System.currentTimeMillis() - startTime >= timeoutMillis) {
                onTimeout.run();
            }
    }


}
