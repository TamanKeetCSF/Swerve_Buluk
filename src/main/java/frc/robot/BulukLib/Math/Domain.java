package frc.robot.BulukLib.Math;

public class Domain{

    private double min, max;
    /**
     * Represents a range zone from x to y value
     * @param min from min
     * @param max to max
     */
    public Domain(double min , double max){
        this.min = min;
        this.max = max;
    }

    public double minValue(){
        return min;
    }
    public double maxValue(){
        return max;
    }

    public boolean inRange(double value){
        return value >= min && value <= max;
    }
}
