package frc.robot.motor;

public class LinearFilter {
    private double maxRateUp, maxRateDown;
    private double value;

    public LinearFilter(double maxRateUp, double maxRateDown) {
        this.maxRateDown = maxRateDown;
        this.maxRateUp = maxRateUp;
    }
    public LinearFilter(double maxRate) {
        this(maxRate, maxRate);
    }

    public void update(double newValue) {
        value = Math.min(value + maxRateUp, Math.max(newValue, value - maxRateDown));
    }

    public double getValue() {
        return value;
    }

    public void setValue(double value) {
        this.value = value;
    }
}
