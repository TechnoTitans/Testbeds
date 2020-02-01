/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.TitanSRX;

public class IntakeSubsystem extends SubsystemBase {
	private final Solenoid piston;
	/**
	 * Creates a new ExampleSubsystem.
	 */
	private TitanSRX intakeMotor;

	private static final double EXPEL_SPEED = 1;
	private static final double INTAKE_SPEED = -1;

	public IntakeSubsystem(TitanSRX intakeMotor, Solenoid piston) {
		this.intakeMotor = intakeMotor;
		this.piston = piston;
	}

	public void stop() { intakeMotor.set(0); }

	public void expel() {
		intakeMotor.set(EXPEL_SPEED);
	}

	public void intake() {
		intakeMotor.set(INTAKE_SPEED);
	}

	public void setSpeed(double speed) {
		intakeMotor.set(speed);
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
