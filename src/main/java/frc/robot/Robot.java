/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
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
	private Command autonomousCommand;

	private RobotContainer robotContainer;

	// todo move to better place
	// todo calc more accurate values

	// todo reverse talonfx encoders by wrapping around

	/**
	 * This method is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		robotContainer = new RobotContainer();

		robotContainer.hoodMotorEncoder.reset();
		robotContainer.shootMotorEncoder.reset();
		robotContainer.zMotorEncoder.reset();
		robotContainer.driveTrain.resetEncoders();

		double turretAngle = 0; //todo get actual angles for both
		double hoodAngle = 0;
//		CommandScheduler.getInstance().schedule(new RotateTurret(turretAngle, robotContainer.turret));
//		CommandScheduler.getInstance().schedule(new RotateHood(hoodAngle, robotContainer.turret));

		CommandScheduler.getInstance().setDefaultCommand(robotContainer.driveTrain, robotContainer.driveTrainCommand);
		CommandScheduler.getInstance().setDefaultCommand(robotContainer.intake, robotContainer.intakeTeleopCommand);
		CommandScheduler.getInstance().setDefaultCommand(robotContainer.turret, robotContainer.turretTeleopCommand);
//		robotContainer.driveTrain.setShifter(true);

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
		CommandScheduler.getInstance().run();

		// todo make everything private that is publicly used here
		SmartDashboard.putNumber("zMotor Motor Encoder", robotContainer.zMotorEncoder.getRawPosition());
		SmartDashboard.putNumber("hoodMotor Motor Encoder", robotContainer.hoodMotorEncoder.getRawPosition());
		SmartDashboard.putNumber("hoodMotor Error", robotContainer.turret.getHoodPositionSetpoint() -
													robotContainer.hoodMotorEncoder.getRawPosition());

//		SmartDashboard.putNumber("Drive Train Encoder LF", robotContainer.leftFrontMotorFX.getSelectedSensorPosition());
//		SmartDashboard.putNumber("Drive Train Encoder RF", robotContainer.rightFrontMotorFX.getSelectedSensorPosition());

		SmartDashboard.putBoolean("Shifter State (True: LOW, False: HIGH)", robotContainer.shifterSolenoid.get());
//		SmartDashboard.putData(robotContainer.intakeSolenoid);
//		SmartDashboard.putData(robotContainer.titanFXCoolingPiston);
		
//		SmartDashboard.putNumber("Xbox Left", robotContainer.oi.getXboxLeftY());
//		SmartDashboard.putNumber("Robot Input", robotContainer.oi.getXboxLeftY());
//		ColorSensorV3.RawColor detectedColor = robotContainer.controlPanel.getColor();
//		SmartDashboard.putNumber("Red value", detectedColor.red);
//		SmartDashboard.putNumber("Green value", detectedColor.green);
//		SmartDashboard.putNumber("Blue value", detectedColor.blue);



//		SmartDashboard.putNumber("Turret zMotor current", robotContainer.zMotor.getCurrent());
//		SmartDashboard.putNumber("Turret hood current", robotContainer.hoodMotor.getCurrent());
//		SmartDashboard.putNumber("Turret flywheel current", robotContainer.shootMotor.getCurrent());
//		SmartDashboard.putNumber("Turret intake current", robotContainer.intakeMotor.getCurrent());

		// drivetrain
		SmartDashboard.putNumber("Falcon Right Front Current", robotContainer.rightFrontMotorFX.getCurrent());
		SmartDashboard.putNumber("Falcon Left Front Current", robotContainer.leftFrontMotorFX.getCurrent());

		SmartDashboard.putNumber("Falcon RF Temp", robotContainer.rightFrontMotorFX.getTemperature());
		SmartDashboard.putNumber("Falcon LF Temp", robotContainer.leftFrontMotorFX.getTemperature());
		SmartDashboard.putNumber("Falcon RB Temp", robotContainer.rightBackMotorFX.getTemperature());
		SmartDashboard.putNumber("Falcon LB Temp", robotContainer.leftBackMotorFX.getTemperature());


		TalonSRXConfiguration zMotorConfig = new TalonSRXConfiguration();
		robotContainer.zMotor.getAllConfigs(zMotorConfig, 0);

//		SmartDashboard.putNumber("Zmotor Config continousCurrentlimit", zMotorConfig.continuousCurrentLimit);
//		SmartDashboard.putNumber("Zmotor Config peak limit", zMotorConfig.peakCurrentLimit);
//		SmartDashboard.putNumber("Zmotor Config duration", zMotorConfig.peakCurrentDuration);



		SmartDashboard.putNumber("Flywheel setpoint (rpm)", robotContainer.turret.getRPMSetpoint());
		SmartDashboard.putNumber("flywheel velocity", robotContainer.shootMotor.getSelectedSensorVelocity() * 600f / 4096);
		SmartDashboard.putNumber("closed loop error ", robotContainer.shootMotor.getClosedLoopError());
		SmartDashboard.putNumber("Left Front Falcon Temperature (C)", robotContainer.leftFrontMotorFX.getTemperature());
		SmartDashboard.putNumber("Left Back Falcon Temperature (C)", robotContainer.leftBackMotorFX.getTemperature());
		SmartDashboard.putNumber("Right Front Falcon Temperature (C)", robotContainer.rightFrontMotorFX.getTemperature());
		SmartDashboard.putNumber("Right Back Falcon Temperature (C)", robotContainer.rightBackMotorFX.getTemperature());
		SmartDashboard.putNumber("Average Motor Temp", (robotContainer.leftFrontMotorFX.getTemperature() + robotContainer.rightFrontMotorFX.getTemperature()) / 2.0);
		SmartDashboard.putBoolean("Cooling on", robotContainer.titanFXCoolingPiston.get());


		SmartDashboard.putNumber("Flywheel setpoint (RPM)", robotContainer.turret.getRPMSetpoint());
		SmartDashboard.putNumber("Flywheel velocity (RPM)", robotContainer.shootMotor.getSelectedSensorVelocity() * (60 * 10) / 4096f);
		SmartDashboard.putNumber("Flywheel Closed Loop Error", robotContainer.shootMotor.getClosedLoopError());


		SmartDashboard.putBoolean("Bottom Hood LS", robotContainer.hoodBottomLS.isPressed());

		SmartDashboard.putBoolean("Has Released Mech", robotContainer.climb.hasReleasedMech());
		SmartDashboard.putNumber("Game Time", robotContainer.climb.getEndgameTime());

		robotContainer.vision.getData();


//		SmartDashboard.putNumber("Vision y-angle (degrees)", robotContainer.vision.getAngleY());
//		SmartDashboard.putNumber("Vision x-angle (degrees)", robotContainer.vision.getAngleX());
//		SmartDashboard.putNumber("Vision distance (in)", robotContainer.vision.getDistance());
//		SmartDashboard.putNumber("Vision Center x", robotContainer.vision.getCenterX());
//		SmartDashboard.putNumber("Vision Center y", robotContainer.vision.getCenterY());


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
		autonomousCommand = robotContainer.getAutonomousCommand();

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

		robotContainer.climb.init();
	}

	/**
	 * This method is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
//		double leftJoyZ = robotContainer.oi.getLeftJoyZ();
//		double flywheelP = (leftJoyZ + 1.0) / 2;
//		robotContainer.shootMotor.config_kP(PIDConstants.kSlotIdx, flywheelP, PIDConstants.kTimeoutMs);
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
