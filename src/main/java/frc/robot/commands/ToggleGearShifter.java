package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TankDrive;

public class ToggleGearShifter extends InstantCommand {


	public ToggleGearShifter(TankDrive driveTrain) {
		super(() -> {
			driveTrain.toggleShifter();
			SmartDashboard.putBoolean("Toggled: ", true);
		}, driveTrain);
	}

}
