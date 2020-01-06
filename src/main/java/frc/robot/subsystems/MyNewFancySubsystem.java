package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * The following class is dummy code used as an example for github.
 */
public class MyNewFancySubsystem extends SubsystemBase {


	private final DigitalInput limitSwitch;
	private final AddressableLED led;

	public MyNewFancySubsystem(DigitalInput limitSwitch, AddressableLED led) {
		this.limitSwitch = limitSwitch;
		this.led = led;
	}

	public void enableLights() {
		if (this.limitSwitch.get()) {
			this.led.start();
		}
	}


}

