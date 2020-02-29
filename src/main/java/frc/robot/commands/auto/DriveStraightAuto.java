package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.TankDrive;

public class DriveStraightAuto extends CommandBase {

	// note! This auto assumes

	public static final double MAX_DISTANCE = 25 * 12; // inches


	private final TankDrive driveTrain;
	private final double desiredDistance;
	private final double desiredSpeed;

	/**
	 *
	 * @param driveTrain drivetrain subsystem
	 * @param distance distance to travel in inches
	 * @param speed desired speed to travel
	 */
	public DriveStraightAuto(TankDrive driveTrain, double distance, double speed) {
		super();
		this.driveTrain = driveTrain;
		this.desiredDistance = MathUtil.clamp(distance, -MAX_DISTANCE, MAX_DISTANCE);
		// if we want to go backwards, the speed must be negative
		this.desiredSpeed = Math.copySign(desiredDistance, speed);
		addRequirements(driveTrain);
	}

	@Override
	public void initialize() {
		this.driveTrain.resetEncoders();
	}

	@Override
	public void execute() {
		this.driveTrain.set(this.desiredSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		this.driveTrain.stop();
	}

	@Override
	public boolean isFinished() {
		return driveTrain.getRightEncoder().getDistance() >= this.desiredDistance;
	}
}
