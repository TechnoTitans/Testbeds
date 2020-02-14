package frc.robot.motor;

import frc.robot.motor.Encoder;

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
