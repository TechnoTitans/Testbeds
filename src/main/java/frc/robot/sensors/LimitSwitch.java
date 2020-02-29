package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch extends DigitalInput{
    private boolean inverted;

    public LimitSwitch(int channel, boolean inverted) {
        super(channel);
        this.inverted = inverted;
    }

    /**
     *
     * @return whether or not the limit switch is deemed "pressed" or active, taking into account normally open/closed
     */
    public boolean isPressed() {
        return this.get() == !inverted;
    }
}
