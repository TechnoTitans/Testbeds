package frc.robot.motors;



public interface Motor {

	void set(double speed);

	double getPercentSpeed();

	double getSpeed();

	void stop();

	void brake();

	void coast();

	boolean hasEncoder();

	Encoder getEncoder();

	// public void setBrakeMode(boolean enable);

	int getChannel();

	boolean isReversed();

	double getCurrent();
}