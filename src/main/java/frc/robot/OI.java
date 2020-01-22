package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {

	private Joystick left, right;
	private XboxController xbox;
	private static final double percentDeadbandThreshold = 0.1;

	private Button btnClimberExtend,
				   btnClimbUp,
			       btnClimbDown,
				   btnPanelSpin,
				   btnPanelArmExtend,
	               btnPanelArmRetract,
			       btnCellIntake,
				   btnCellOuttake,
				   btnTarget,
				   btnShoot;

	public OI() { initialize();}

	private static class Btn extends JoystickButton {
		private GenericHID hid;
		private int buttonNumber;

		public Btn(GenericHID hid, int buttonNumber) {
			super(hid, buttonNumber);
			this.hid = hid;
			this.buttonNumber = buttonNumber;
		}

		public boolean isPressed() {
			return hid.getRawButtonPressed(buttonNumber);
		}

		public boolean isReleased() {
			return hid.getRawButtonReleased(buttonNumber);
		}

		public boolean isHeld() {
			return get();
		}
	}

	private void initialize() {

		left = new Joystick(RobotMap.LEFT_JOYSTICK);
		right = new Joystick(RobotMap.RIGHT_JOYSTICK);
		xbox = new XboxController(RobotMap.AUX_JOYSTICK_1);

		btnClimberExtend = new Btn(left, 4);
		btnClimbUp = new Btn(left,11);
		btnClimbDown = new Btn(left, 12);
		btnCellIntake = new Btn(left, 6);
		btnCellOuttake = new Btn(left, 7);
		btnPanelArmExtend = new Btn(left, 8);
		btnPanelArmRetract = new Btn(left, 9);
		btnPanelSpin = new Btn(left, 10);
		btnTarget = new Btn(left, 11);
	}



}
