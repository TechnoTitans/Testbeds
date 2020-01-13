/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	/**
	 * Creates a new ExampleSubsystem.
	 */
	private TitanSRX intakeMotor;

	private static final double ExpelSpeed = 1;
	private static final double IntakeSpeed = -1;

	public IntakeSubsystem(TitanSRX intakeMotor) {
		this.intakeMotor = intakeMotor;
	}

	public void expel() {
		intakeMotor.set(ExpelSpeed);
	}

	public void stop() {
		intakeMotor.set(0);
	}

	public void intake() {
		intakeMotor.set(IntakeSpeed);
	}

	public void hold() {
		intakeMotor.set(IntakeSpeed / 4);
	}

	public void setGrabberMotor(double speed) {
		intakeMotor.set(speed);
	}

	/**
	 * Will be called periodically whenever the CommandScheduler runs.
	 */
	@Override
	public void periodic() {

	}
}
