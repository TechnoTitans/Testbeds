package frc.robot.motor;

public class Filter {
	private double sensitivity;
	private double value = 0;

	public Filter(double sensitivity) {
		this.sensitivity = sensitivity;
	}

	public void update(double newValue) {
		value = sensitivity * newValue + (1 - sensitivity) * value;
	}

	public double getValue() {
		return value;
	}

	public void setValue(double value) {
		this.value = value;
	}
}
