/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.motor.Filter;
import frc.robot.motor.TitanSRX;

public class IntakeSubsystem extends SubsystemBase {
	private final Solenoid piston;
	/**
	 * Creates a new ExampleSubsystem.
	 */
	private TitanSRX intakeMotor;

	private static final double EXPEL_SPEED = -.7;
	private static final double INTAKE_SPEED = .5;
	private double currentSpeed = 0;
	private final Filter intakeMotorFilter;

	public IntakeSubsystem(TitanSRX intakeMotor, Solenoid piston) {
		this.intakeMotor = intakeMotor;
		this.piston = piston;
		intakeMotorFilter = new Filter(0.7);
	}

	public void stop() {
		currentSpeed = 0;
		intakeMotorFilter.setValue(0);
		this.intakeMotor.stop();
	}

	public void expel() {
		currentSpeed = EXPEL_SPEED;
	}

	public void intake() {
		currentSpeed = INTAKE_SPEED;
	}

	public void setSpeed(double speed) {
		speed = MathUtil.clamp(speed, EXPEL_SPEED, INTAKE_SPEED);
		this.currentSpeed = speed;
	}

	public void toggleIntakeMotors() {
		boolean isRunning = currentSpeed != 0;
		SmartDashboard.putBoolean("Intake Motor Running", this.intakeMotor.get() > 0);
		if (isRunning) {
			this.stop();
		} else {
			this.currentSpeed = INTAKE_SPEED;
		}
	}

	/**
	 * must be called repeatedly
	 */
	public void run() {
		intakeMotorFilter.update(currentSpeed);
		intakeMotor.set(intakeMotorFilter.getValue());
	}


	/**
	 * Will be called periodically whenever the CommandScheduler runs.
	 */
	@Override
	public void periodic() {

	}

	public void togglePiston() {
		this.piston.set(!this.piston.get());
	}
}
