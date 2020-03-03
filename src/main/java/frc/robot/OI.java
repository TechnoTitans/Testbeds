package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class OI {

	public static final int XBOX_BUMPER_RIGHT = 6;
	public static final int XBOX_BUMPER_LEFT = 5;
	public static final int XBOX_BTN_SELECT = 7;
	public static final int XBOX_BTN_START = 8;

	private static final double PERCENT_DEADBAND_THRESHOLD = 0.1;


	private Joystick leftJoystick;
	private Joystick rightJoystick;
	private XboxController xbox;


	public OI() { initialize();}

	private void initialize() {
		leftJoystick = new Joystick(RobotMap.LEFT_JOYSTICK);
		rightJoystick = new Joystick(RobotMap.RIGHT_JOYSTICK);
		xbox = new XboxController(RobotMap.AUX_JOYSTICK_1);
	}

	public Joystick getLeftJoystick() {
		return leftJoystick;
	}

	public Joystick getRightJoystick() {
		return rightJoystick;
	}

	public double getLeftJoyY() {
		return deadband(-leftJoystick.getY(), PERCENT_DEADBAND_THRESHOLD);
	}

	public double getRightJoyY() {
		return deadband(-rightJoystick.getY(), PERCENT_DEADBAND_THRESHOLD);
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


	public double getLeftJoyZ() {
		return leftJoystick.getZ();
	}


}
