package frc.robot.sensors;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class TitanButton extends JoystickButton {
    private GenericHID hid;
    private int buttonNumber;

    public TitanButton(GenericHID hid, int buttonNumber) {
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
