/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * methods corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	Command autonomousCommand = new autonomousCommand();

	private RobotContainer robotContainer;

	public static TitanSRX mainMotor;

	/**
	 * This method is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		robotContainer = new RobotContainer();

		autonomousCommand = new autonomousCommand();

	}

	/**
	 * This method is called every robot packet, no matter the mode. Use this for items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		// 44:pulses per revolution, 34:1 gearbox
		SmartDashboard.putNumber("Encoder Value", robotContainer.mainMotorEncoder.getRawPosition());
		SmartDashboard.putNumber("Encoder Value Degrees", robotContainer.mainMotorEncoder.getRawPosition() * 360f / (44 * 34f));
		SmartDashboard.putNumber("Encoder Value Rotations", robotContainer.mainMotorEncoder.getRawPosition() / (44 * 34f));
		SmartDashboard.putNumber("P Constant", robotContainer.pConstant);
		CommandScheduler.getInstance().run();
	}

	/**
	 * This method is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {


		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();

		}
	}

	/**
	 * This method is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.

		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}



	}

	/**
	 * This method is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
//		double leftJoyZ = robotContainer.oi.getLeftJoyZ();
//		double flywheelP = (leftJoyZ + 1.0) / 2;
//		robotContainer.flywheelMotor.config_kP(PIDConstants.kSlotIdx, flywheelP, PIDConstants.kTimeoutMs);
//		SmartDashboard.putNumber("Flywheel P", flywheelP);
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/**
	 * This method is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}
}
