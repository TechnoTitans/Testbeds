package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.commands.HopperIntake;
import frc.robot.motor.Filter;
import frc.robot.subsystems.*;

public class ShootThenDriveStraightAuto extends CommandBase {

	// note! This auto assumes

	public static final double MAX_DISTANCE = 25 * 12; // inches


	private final TankDrive driveTrain;
	private final double desiredDistance;
	private final double desiredSpeed;
	private final Filter speedFilter;


	private final TurretSubsystem turret;
	private final HopperSubsystem hopper;
	private final FeederSubsystem feeder;
	private final IntakeSubsystem intake;

	private final Timer timer;

//	private final Command hopperCommand;

	/**
	 * @param driveTrain drivetrain subsystem
	 * @param distance distance to travel in inches
	 * @param speed desired speed to travel
	 */
	public ShootThenDriveStraightAuto(TankDrive driveTrain, double distance, double speed,
									  TurretSubsystem turret, HopperSubsystem hopper, FeederSubsystem feeder,
									  IntakeSubsystem intake) {
		super();
		this.driveTrain = driveTrain;
		this.desiredDistance = MathUtil.clamp(distance, -MAX_DISTANCE, MAX_DISTANCE);
		// if we want to go backwards, the speed must be negative
		this.desiredSpeed = Math.copySign(speed, desiredDistance);
		this.speedFilter = new Filter(0.5);
		this.speedFilter.update(0); // start the filter off at zero

		this.turret = turret;
		this.hopper = hopper;
		this.feeder = feeder;
		this.intake = intake;
//		this.hopperCommand = new HopperAuto(hopper);

		timer = new Timer();

		addRequirements(driveTrain);
		SmartDashboard.putNumber("desired distance", desiredDistance);
		SmartDashboard.putNumber("desired speed", desiredSpeed);
	}

	@Override
	public void initialize() {
		this.driveTrain.resetEncoders();
		this.speedFilter.setValue(0); // reset the filter so that subsequent autonomouses work

		timer.start();
		intake.togglePiston();

//		btnToggleHopperIntake.whileHeld(new HopperIntake(hopper));
//		this.turret.
	}

	@Override
	public void execute() {
		speedFilter.update(this.desiredSpeed);
		// TODO Filter setpoint at the TitanFX class level so that we don't have to keep filtering stuff

		if (timer.hasElapsed(7) && timer.get() < 11) {
			hopper.expel();
		}

//		hopper.intake();
		if(timer.get() >= 11) {
			feeder.setBelt(0);
			hopper.stop();
//			intake.setSpeed(0);
			intake.stop();
			this.turret.setAutoPreset(0);

			this.driveTrain.set(speedFilter.getValue());
		} else {
			this.turret.setAutoPreset(2);
			feeder.setBelt(1);
//			intake.setSpeed(1);
			intake.intake();
			intake.run();
		}

		SmartDashboard.putNumber("Right encoder value", driveTrain.getRightEncoder().getDistance());
		SmartDashboard.putNumber("Right encoder value", driveTrain.getRightEncoder().getDistance());
		SmartDashboard.putNumber("DriveTrain percent output", driveTrain.getRight().get());
	}

	@Override
	public void end(boolean interrupted) {
		this.driveTrain.stop();
	}

	@Override
	public boolean isFinished() {
		// TODO Enforce max distance here with condition
		if (Math.signum(this.desiredDistance) == -1) {
			return driveTrain.getRightEncoder().getDistance() <= this.desiredDistance;

		} else {
			return driveTrain.getRightEncoder().getDistance() >= this.desiredDistance;
		}
	}
}
