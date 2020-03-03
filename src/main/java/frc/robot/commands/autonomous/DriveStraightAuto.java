package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.motor.Filter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.TankDrive;

public class DriveStraightAuto extends CommandBase {


	public static final double MAX_DISTANCE = 25 * 12; // inches


	private final TankDrive driveTrain;
	private final double desiredDistance;
	private final double desiredSpeed;
	private final Filter speedFilter;

	/**
	 *
	 * @param driveTrain drivetrain subsystem
	 * @param distance distance to travel in inches
	 * @param speed desired speed to travel
	 */
	public DriveStraightAuto(TankDrive driveTrain, double distance, double speed) {
		super();
		this.driveTrain = driveTrain;
//		this.desiredDistance = distance;
		this.desiredDistance = MathUtil.clamp(distance, -MAX_DISTANCE, MAX_DISTANCE);
		// if we want to go backwards, the speed must be negative
//		this.desiredSpeed = speed;
		this.desiredSpeed = Math.copySign(speed, desiredDistance);
		this.speedFilter = new Filter(0.5);
		this.speedFilter.update(0); // start the filter off at zero
		addRequirements(driveTrain);
		SmartDashboard.putNumber("desired distance", desiredDistance);
		SmartDashboard.putNumber("desired speed", desiredSpeed);
	}

	@Override
	public void initialize() {
		this.driveTrain.resetEncoders();
		// todo test enforcing low shifter
//		this.driveTrain.setShifter(TankDrive.SHIFT_LOW_TORQUE);	// note! This auto assumes high speed mode, not high power mode. we will ensure this here
		this.speedFilter.setValue(0); // reset the filter so that subsequent autonomouses work
	}

	@Override
	public void execute() {
		speedFilter.update(this.desiredSpeed);
		// TODO Filter setpoint at the TitanFX class level so that we don't have to keep filtering stuff
		this.driveTrain.set(speedFilter.getValue());
		SmartDashboard.putNumber("Right encoder value", driveTrain.getRightEncoder().getDistance());
		SmartDashboard.putNumber("DriveTrain percentoutput", driveTrain.getRight().get());
	}

	@Override
	public void end(boolean interrupted) {
		this.driveTrain.stop();
	}

	@Override
	public boolean isFinished() {
		// TODO Enforce max distance here with condition
		final double distanceTraveled = driveTrain.getRightEncoder().getDistance();
		if (Math.signum(this.desiredDistance) == -1) {
			// todo test out traveling max distance
			return (distanceTraveled <= this.desiredDistance) || distanceTraveled <= -MAX_DISTANCE;
		} else {
			return distanceTraveled >= this.desiredDistance || distanceTraveled >= MAX_DISTANCE;
		}
	}
}
