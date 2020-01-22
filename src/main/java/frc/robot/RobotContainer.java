/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.motor.TitanSRX;
import frc.robot.motors.TitanFX;
import frc.robot.sensors.QuadEncoder;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Declare the robot's components here
    // see

    // The robot's subsystems and commands are defined here...

    private TitanSRX shootMotor;
    private TitanSRX zMotor;
    private TitanSRX hoodMotor;
    private TitanSRX beltMotor;
    private TitanSRX spinningMotor;
    private ColorSensorV3 colorSensor;
    public static TurretSubsystem turret;
    public ControlPanelSubsystem controlPanel;
    private CommandBase autonomousCommand;
    private TitanFX leftFrontMotorFX;
    private TitanFX leftBackMotorFX;
    private TitanFX rightFrontMotorFX;
    private TitanFX rightBackMotorFX;
    private DriveTrain driveTrain;


    private OI oi;


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        shootMotor = new TitanSRX(0, false);
        zMotor = new TitanSRX(0, false);
        hoodMotor = new TitanSRX(0, false);
        beltMotor = new TitanSRX(0, false);
        zMotor.setEncoder(new QuadEncoder(zMotor, 0, false));
        turret = new TurretSubsystem(shootMotor, zMotor, hoodMotor, beltMotor);
        spinningMotor = new TitanSRX(0, false);
        colorSensor = new ColorSensorV3(RobotMap.COLOR_SENSOR_PORT);
        controlPanel = new ControlPanelSubsystem(spinningMotor, colorSensor);

        leftFrontMotorFX = new TitanFX(0, false);
        leftBackMotorFX = new TitanFX(0, false);
        rightFrontMotorFX = new TitanFX(0, false);
        rightBackMotorFX = new TitanFX(0, false);
        leftBackMotorFX.follow(leftFrontMotorFX);
        rightBackMotorFX.follow(rightFrontMotorFX);
    	driveTrain = new TankDrive(leftFrontMotorFX, rightFrontMotorFX);
        autonomousCommand = new DriveTrainCommand(driveTrain);
        // Configure the button bindings
        configureButtonBindings();

    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
     */
    private void configureButtonBindings() {
        oi = new OI();
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autonomousCommand;
    }
}
