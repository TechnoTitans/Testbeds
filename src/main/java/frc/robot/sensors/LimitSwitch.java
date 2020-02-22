package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch extends DigitalInput{
    private boolean inverted;

    public LimitSwitch(int channel, boolean inverted) {
        super(channel);
        this.inverted = inverted;
    }

    public boolean isPressed() {
        return this.get() == !inverted;
    }
}
