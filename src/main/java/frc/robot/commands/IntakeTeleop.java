package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeTeleop extends CommandBase {
	private final IntakeSubsystem intake;
	private final DoubleSupplier rollerInput;

	// todo filtering
	public IntakeTeleop(DoubleSupplier rollerInput, IntakeSubsystem intake) {
		super();
		this.intake = intake;
		this.rollerInput = rollerInput;
		addRequirements(intake);
	}

	@Override
	public void execute() {
		this.intake.setSpeed(rollerInput.getAsDouble());
	}

	@Override
	public void end(boolean interrupted) {
		this.intake.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
