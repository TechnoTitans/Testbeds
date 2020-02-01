package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class OI {

	public static final int BTNNUM_TOGGLE_SHIFTER = 4;

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
	public double getLeft() {
		return leftJoystick.getY(Hand.kLeft);
	}

	public double getRight() {
		return rightJoystick.getY(Hand.kRight);
	}



}
