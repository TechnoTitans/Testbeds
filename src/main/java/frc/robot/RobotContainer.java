/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.motor.TitanSRX;
import frc.robot.sensors.QuadEncoder;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("FieldCanBeLocal")
public class RobotContainer {

	/**
	 * The container for the robot.  Contains subsystems, OI devices, and commands.
	 */

	//Motors
	TitanSRX leftMotor;
	TitanSRX rightMotor;

	//Encoders
	QuadEncoder leftMotorEncoder;
	QuadEncoder rightMotorEncoder;

	//Subsystems
	TankDrive driveTrain;

	//Sensors
	ColorSensorV3 colorSensor;


	public RobotContainer() {
		//motors and encoders
		leftMotor = new TitanSRX(RobotMap.LEFT_TALON, false); //could be true
		rightMotor = new TitanSRX(RobotMap.RIGHT_TALON, true); //could be false
		leftMotorEncoder = new QuadEncoder(leftMotor, TankDrive.DRIVETRAIN_INCHES_PER_PULSE,false); //could be true
		rightMotorEncoder = new QuadEncoder(rightMotor, TankDrive.DRIVETRAIN_INCHES_PER_PULSE, true); //could be false
		leftMotor.setEncoder(leftMotorEncoder);
		rightMotor.setEncoder(rightMotorEncoder);

		//subsystems
		driveTrain = new TankDrive(leftMotor, rightMotor);

		//sensors
		colorSensor = new ColorSensorV3(RobotMap.COLOR_SENSOR_PORT);
	}

	/**
	 * Use this method to define your button->command mappings.  Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}), and then passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
	 */
	private void configureButtonBindings() {

	}

}



