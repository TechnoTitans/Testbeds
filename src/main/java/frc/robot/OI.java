package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class OI {

	public static final int XBOX_BUMPER_RIGHT = 6;
	public static final int XBOX_BUMPER_LEFT = 5;
	public static final int BTNNUM_TURRET_AUTO_AIM = 8;
	public Joystick leftJoystick, rightJoystick;
	private XboxController xbox;
	private static final double percentDeadbandThreshold = 0.1;


	public OI() { initialize();}

	private void initialize() {
		leftJoystick = new Joystick(RobotMap.LEFT_JOYSTICK);
		rightJoystick = new Joystick(RobotMap.RIGHT_JOYSTICK);
		xbox = new XboxController(RobotMap.AUX_JOYSTICK_1);
	}



	// todo verify this works
	public double getLeftJoyY() {
		return deadband(leftJoystick.getY(), 0.1); // todo more readble
	}

	public double getRightJoyY() {
		return deadband(rightJoystick.getY(), 0.1);
	}

	public double getXboxLeftY() {
		return -xbox.getY(Hand.kLeft);
	}

	public double getXboxRightY() {
		return xbox.getY(Hand.kRight);
	}

	public double getXboxRightX() {
		return xbox.getX(Hand.kRight);
	}

	public double getXboxLeftTrigger() {
		return xbox.getTriggerAxis(Hand.kLeft);
	}

	public XboxController getXbox() {
		return xbox;
	}

	public double deadband(double value, double deadband) {
		if (-deadband <= value && value <= deadband) {
			value = 0;
		} else if (value > deadband){
			value -= deadband;
			value *= (1 + deadband);
		} else if (value < -deadband){
			value += deadband;
			value *= (1 + deadband);
		}
		return value;
	}


}
