/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.motor.TitanSRX;
import frc.robot.motor.TitanFX;
import frc.robot.sensors.QuadEncoder;
import frc.robot.sensors.TitanButton;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Declare the robot's components here

    // The robot's subsystems and commands are defined here...
    private DigitalInput beltLimitSwitch;
    private Solenoid shifterSolenoid;
    private final Solenoid intakeSolenoid;

    private TitanSRX shootMotor;
    private TitanSRX zMotor;
    private TitanSRX hoodMotor;
    private TitanSRX beltMotor;
    private TitanSRX spinningMotor;
    private TitanSRX intakeMotor;

    private ColorSensorV3 colorSensor;
	public IntakeSubsystem intake;
    private TitanFX leftFrontMotorFX;
    private TitanFX leftBackMotorFX;
    private TitanFX rightFrontMotorFX;
    private TitanFX rightBackMotorFX;

    // MARK - Subsystem Declarations
    public TurretSubsystem turret;
    public ControlPanelSubsystem controlPanel;
    public TankDrive driveTrain;

    // MARK - Command declarations
    public DriveTrainCommand driveTrainCommand;
    public ToggleGearShifter toggleGearShifterCommand;
    public IntakeTeleop intakeTeleopCommand;
    public RotateTurretTeleop rotateTurretTeleop;


    private CommandBase autonomousCommand;

    private OI oi;
    private TitanButton btnToggleShifter;
    private TitanButton btnToggleIntake;
    private TitanButton btnHoodDown;
    private TitanButton btnHoodUp;
    private TitanButton btnTurretLeft;
    private TitanButton btnTurretRight;
    private TitanButton btnShoot;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        oi = new OI();
        shootMotor = new TitanSRX(0, false);
        zMotor = new TitanSRX(0, false);
        hoodMotor = new TitanSRX(0, false);
        beltMotor = new TitanSRX(0, false);
        zMotor.setEncoder(new QuadEncoder(zMotor, 0, false));
        beltLimitSwitch = new DigitalInput(0);
        turret = new TurretSubsystem(shootMotor, zMotor, hoodMotor, beltMotor, beltLimitSwitch);
        spinningMotor = new TitanSRX(0, false);
        colorSensor = new ColorSensorV3(RobotMap.COLOR_SENSOR_PORT);
        controlPanel = new ControlPanelSubsystem(spinningMotor, colorSensor);

        leftFrontMotorFX = new TitanFX(RobotMap.LEFT_TALON_FRONT, RobotMap.REVERSED_LF_TALON);
        leftBackMotorFX = new TitanFX(RobotMap.LEFT_TALON_BACK, RobotMap.REVERSED_LB_TALON);
        rightFrontMotorFX = new TitanFX(RobotMap.RIGHT_TALON_FRONT, RobotMap.REVERSED_RF_TALON);
        rightBackMotorFX = new TitanFX(RobotMap.RIGHT_TALON_BACK, RobotMap.REVERSED_RB_TALON);
        leftBackMotorFX.follow(leftFrontMotorFX); // todo set titanfx motor encoders
        rightBackMotorFX.follow(rightFrontMotorFX);
        shifterSolenoid = new Solenoid(RobotMap.GEAR_SHIFT_SOLENOID);
        driveTrain = new TankDrive(leftFrontMotorFX, rightFrontMotorFX, shifterSolenoid);

        intakeMotor = new TitanSRX(RobotMap.INTAKE_MOTOR, RobotMap.REVERSED_INTAKE_MOTOR);
        intakeSolenoid = new Solenoid(RobotMap.INTAKE_SOLENOID);
        intake = new IntakeSubsystem(intakeMotor, intakeSolenoid);

        // MARK - command initialization
        driveTrainCommand = new DriveTrainCommand(oi::getLeft, oi::getRight, driveTrain, false);
        toggleGearShifterCommand = new ToggleGearShifter(driveTrain);
        intakeTeleopCommand = new IntakeTeleop(oi::getXboxLeft , intake);
        rotateTurretTeleop = new RotateTurretTeleop(oi::getXboxLeft, oi::getXboxRight, turret);
        autonomousCommand = new InstantCommand(); // a do nothing command for now



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
        // MARK - button definitions
        btnToggleShifter = new TitanButton(oi.leftJoystick, OI.BTNNUM_TOGGLE_SHIFTER);
//        btnToggleIntake = new TitanButton(oi.leftJoystick, OI.BTNNUM_TOGGLE_INTAKE);

        // MARK - bindings
        btnToggleShifter.whenPressed(new ToggleGearShifter(driveTrain));
//        btnToggleIntake.whenPressed(new ToggleIntake(intake));
        btnShoot = new TitanButton(oi.getXbox(), OI.BTNNUM_SHOOT);

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



