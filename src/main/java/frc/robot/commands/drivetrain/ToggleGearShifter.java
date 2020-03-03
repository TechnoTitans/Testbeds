package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TankDrive;

public class ToggleGearShifter extends InstantCommand {


	public ToggleGearShifter(TankDrive driveTrain) {
		super(driveTrain::toggleShifter, driveTrain);
	}

}
