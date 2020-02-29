package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.TitanVictor;

public class FeederSubsystem extends SubsystemBase {
	private final TitanVictor feederMotor;
	//  todo max speed

	public FeederSubsystem(TitanVictor feederMotor) {
		this.feederMotor = feederMotor;
	}

	public void setBelt(double speed) {
		this.feederMotor.set(speed);
	}

}