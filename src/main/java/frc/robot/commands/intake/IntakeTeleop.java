package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeTeleop extends CommandBase {

	private boolean pressedLast = false;

	private final DoubleSupplier rollerInput;
	private final Button intakeToggleBtn;

	private final IntakeSubsystem intake;

	public IntakeTeleop(IntakeSubsystem intake, DoubleSupplier rollerInput, Button intakeToggleButton) {
		super();
		this.intake = intake;
		this.rollerInput = rollerInput;
		this.intakeToggleBtn = intakeToggleButton;
		addRequirements(intake);
	}

	@Override
	public void execute() {
		this.intake.setSpeed(rollerInput.getAsDouble());

		boolean isPressed = this.intakeToggleBtn.get();
		if (!this.pressedLast && isPressed) {
			intake.togglePiston();
//			intake.toggleIntakeMotors();
		}
		this.pressedLast = isPressed;
		intake.run(); // must be called repeatedly
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
