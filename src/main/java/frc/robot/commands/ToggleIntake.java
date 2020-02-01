package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntake extends InstantCommand {

	public ToggleIntake(IntakeSubsystem intake) {
		super(() -> intake.togglePiston(), intake);
	}

}
