/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.*;

import frc.robot.motor.TitanSRX;
import frc.robot.motor.TitanFX;
import frc.robot.motor.TitanVictor;
import frc.robot.sensors.LimitSwitch;
import frc.robot.sensors.QuadEncoder;
import frc.robot.sensors.TitanButton;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Compressor;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final Compressor compressor;
    private final TitanVictor feederMotor;
    private final FeederSubsystem feeder;
    public final Solenoid titanFXCoolingPiston;

    public final QuadEncoder zMotorEncoder;
    public final QuadEncoder hoodMotorEncoder;
    public final QuadEncoder shootMotorEncoder;
    public final QuadEncoder leftFrontMotorEncoder;
    public final QuadEncoder rightFrontMotorEncoder;
    public final QuadEncoder rightBackMotorEncoder;
    public final QuadEncoder leftBackMotorEncoder;

    // Declare the robot's components here

    public Solenoid shifterSolenoid;
    public final Solenoid intakeSolenoid;
    private final Solenoid climbMechPiston;
    private final Solenoid colorMechPiston;

    public TitanSRX shootMotor;
    private TitanVictor subShootMotor;
    public TitanSRX zMotor;
    public TitanSRX hoodMotor;
    private TitanSRX spinningMotor;
    public TitanSRX intakeMotor;
    private TitanVictor hopperMotor;

    private final LimitSwitch leftTurretLS;
    private final LimitSwitch rightTurretLS;


    private ColorSensorV3 colorSensor;
	public IntakeSubsystem intake;
	public HopperSubsystem hopper;
    public TitanFX leftFrontMotorFX;
    public TitanFX leftBackMotorFX;
    public TitanFX rightFrontMotorFX;
    public TitanFX rightBackMotorFX;

    // MARK - Subsystem Declarations
    public TurretSubsystem turret;
    public ControlPanelSubsystem controlPanel;

    private DifferentialDriveKinematics kinematics;

    public TankDrive driveTrain;

    // MARK - Command declarations
    public DriveTrainCommand driveTrainCommand;
    public ToggleGearShifter toggleGearShifterCommand;
    public IntakeTeleop intakeTeleopCommand;
    public TurretTeleop turretTeleop;

    private CommandBase autonomousCommand;

    // MARK - Operator Interface + Buttons
    public OI oi;
    private TitanButton btnToggleShifter;
    private TitanButton btnToggleIntake;
    private TitanButton btnIncreaseShooterSpeed;
    private TitanButton btnDecreaseShooterSpeed;
    private TitanButton btnToggleHopperIntake;
    private TitanButton btnToggleHopperExpel;
    private TitanButton btnFeederExpel;
    // todo find actual values
    private final double kS = 0; //volts
    private final double kV = 0; //volt-seconds/meter
    private final double kA = 0; // volt-seconds squared/meter
    private final double MAX_VOLTAGE = 0; // volts
    private final double MAX_VELOCITY = 0; //m/s
    private final double MAX_ACCELERATION = 0; //m/s^2
    private final double RAMSETE_B = 2;
    private final double RAMSETE_ZETA = 0.7;
    private final double DRIVE_WIDTH = 0.5; //meters
    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        oi = new OI();
        compressor = new Compressor(RobotMap.COMPRESSOR_ID);

        shootMotor = new TitanSRX(RobotMap.FLYWHEEL1, RobotMap.REVERSED_FLYWHEEL1);
        subShootMotor = new TitanVictor(RobotMap.FLYWHEEL2, RobotMap.REVERSED_FLYWHEEL2);
        subShootMotor.follow(shootMotor);
        zMotor = new TitanSRX(RobotMap.TURRET_ROTATION, RobotMap.REVERSED_TURRET_ROTATION);
        hoodMotor = new TitanSRX(RobotMap.HOOD, RobotMap.REVERSED_HOOD);

        zMotorEncoder = new QuadEncoder(zMotor, 0, false);
        zMotor.setEncoder(zMotorEncoder);

        hoodMotorEncoder = new QuadEncoder(hoodMotor, 0, false);
        hoodMotor.setEncoder(hoodMotorEncoder);

        shootMotorEncoder = new QuadEncoder(shootMotor, 0, false); // todo rename shootmotor to flywheel motor
        shootMotor.setEncoder(shootMotorEncoder);

        leftTurretLS = new LimitSwitch(RobotMap.LEFT_TURRET_LS, RobotMap.LEFT_TURRET_LS_INVERTED);
        rightTurretLS = new LimitSwitch(RobotMap.RIGHT_TURRET_LS, RobotMap.RIGHT_TURRET_LS_INVERTED);
        turret = new TurretSubsystem(shootMotor, subShootMotor, zMotor, hoodMotor, leftTurretLS, rightTurretLS);
        spinningMotor = new TitanSRX(RobotMap.COLOR_WHEEL_MOTOR, RobotMap.REVERSED_COLOR_WHEEL);
//        colorSensor = new ColorSensorV3(RobotMap.COLOR_SENSOR_PORT);
//        controlPanel = new ControlPanelSubsystem(spinningMotor, colorSensor);

        leftFrontMotorFX = new TitanFX(RobotMap.LEFT_TALON_FRONT, RobotMap.REVERSED_LF_TALON);
        leftBackMotorFX = new TitanFX(RobotMap.LEFT_TALON_BACK, RobotMap.REVERSED_LB_TALON);
        rightFrontMotorFX = new TitanFX(RobotMap.RIGHT_TALON_FRONT, RobotMap.REVERSED_RF_TALON);
        rightBackMotorFX = new TitanFX(RobotMap.RIGHT_TALON_BACK, RobotMap.REVERSED_RB_TALON);

        leftFrontMotorEncoder = new QuadEncoder(leftFrontMotorFX, TankDrive.DRIVETRAIN_INCHES_PER_PULSE, true);
        leftFrontMotorFX.setEncoder(leftFrontMotorEncoder);
        rightFrontMotorEncoder = new QuadEncoder(rightFrontMotorFX, TankDrive.DRIVETRAIN_INCHES_PER_PULSE, true);
        rightFrontMotorFX.setEncoder(rightFrontMotorEncoder);
        rightBackMotorEncoder = new QuadEncoder(rightBackMotorFX, TankDrive.DRIVETRAIN_INCHES_PER_PULSE, true);
        rightBackMotorFX.setEncoder(rightBackMotorEncoder);
        leftBackMotorEncoder = new QuadEncoder(leftBackMotorFX, TankDrive.DRIVETRAIN_INCHES_PER_PULSE, true);
        leftBackMotorFX.setEncoder(leftBackMotorEncoder);


        leftBackMotorFX.follow(leftFrontMotorFX);
        rightBackMotorFX.follow(rightFrontMotorFX);

        kinematics = new DifferentialDriveKinematics(DRIVE_WIDTH);

        shifterSolenoid = new Solenoid(RobotMap.COMPRESSOR_ID, RobotMap.GEAR_SHIFT_SOLENOID);
        driveTrain = new TankDrive(leftFrontMotorFX, rightFrontMotorFX, shifterSolenoid);


        intakeMotor = new TitanSRX(RobotMap.INTAKE_MOTOR, RobotMap.REVERSED_INTAKE_MOTOR);
        intakeSolenoid = new Solenoid(RobotMap.COMPRESSOR_ID, RobotMap.INTAKE_SOLENOID);
        intake = new IntakeSubsystem(intakeMotor, intakeSolenoid);

        hopperMotor = new TitanVictor(RobotMap.HOPPER_MOTOR, RobotMap.REVERSED_HOPPER_MOTOR);
        hopper = new HopperSubsystem(hopperMotor);

        feederMotor = new TitanVictor(RobotMap.FEEDER_MOTOR, RobotMap.REVERSED_FEEDER);
        feeder = new FeederSubsystem(feederMotor);

        titanFXCoolingPiston = new Solenoid(RobotMap.COMPRESSOR_ID, RobotMap.FALCON_COOLING_PORT);

        // todo move and put into actual subsystem
        climbMechPiston = new Solenoid(RobotMap.COMPRESSOR_ID, RobotMap.CLIMB_MECH_PISTON);
        colorMechPiston = new Solenoid(RobotMap.COMPRESSOR_ID, RobotMap.COLOR_MECH_PISTON);

        // MARK - Talon/Victor Setup and Configuration

        // WE MUST EXPLICITLY RESET THE SETTINGS ON EACH BECAUSE THEY PERSIST BY DEFAULT
        // todo do this for the rest of the motors.
        shootMotor.configFactoryDefault();
        intakeMotor.configFactoryDefault();
        leftFrontMotorFX.configFactoryDefault();
        leftBackMotorFX.configFactoryDefault();
        rightFrontMotorFX.configFactoryDefault();
        rightBackMotorFX.configFactoryDefault();
        hoodMotor.configFactoryDefault();
        zMotor.configFactoryDefault();

        shootMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                PIDConstants.kPIDLoopIdx,
                PIDConstants.kTimeoutMs);
        /* Config the peak and nominal outputs */
        shootMotor.configNominalOutputForward(0, PIDConstants.kTimeoutMs);
        shootMotor.configNominalOutputReverse(0, PIDConstants.kTimeoutMs);
        shootMotor.configPeakOutputForward(1, PIDConstants.kTimeoutMs);
        shootMotor.configPeakOutputReverse(-1, PIDConstants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        shootMotor.config_kF(PIDConstants.kSlotIdx, PIDConstants.Shooter_Velocity_Gains.kF, PIDConstants.kTimeoutMs);
        shootMotor.config_kP(PIDConstants.kSlotIdx, PIDConstants.Shooter_Velocity_Gains.kP, PIDConstants.kTimeoutMs);
        shootMotor.config_kI(PIDConstants.kSlotIdx, PIDConstants.Shooter_Velocity_Gains.kI, PIDConstants.kTimeoutMs);
        shootMotor.config_kD(PIDConstants.kSlotIdx, PIDConstants.Shooter_Velocity_Gains.kD, PIDConstants.kTimeoutMs);

        /*
        shootMotor.setupCurrentLimiting(5, 0, 0);
        intakeMotor.setupCurrentLimiting(5, 0, 0);
        leftFrontMotorFX.setupCurrentLimiting(5, 6, 50);
        leftBackMotorFX.setupCurrentLimiting(5, 6, 50);
        rightFrontMotorFX.setupCurrentLimiting(5, 6, 50);
        rightBackMotorFX.setupCurrentLimiting(5, 6, 50);
        hoodMotor.setupCurrentLimiting(4,0,0);
        zMotor.setupCurrentLimiting(3,4,100);
        */

        // MARK - command initialization
        driveTrainCommand = new DriveTrainCommand(oi::getLeftJoyY, oi::getRightJoyY, driveTrain, true);
        toggleGearShifterCommand = new ToggleGearShifter(driveTrain);

        intakeTeleopCommand = new IntakeTeleop(oi::getXboxLeftY, intake);
        turretTeleop = new TurretTeleop(oi::getXboxRightX, oi::getXboxRightY, turret, true);

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
        btnToggleShifter = new TitanButton(oi.leftJoystick, 4);
        TitanButton btnToggleColorMechPiston = new TitanButton(oi.leftJoystick, 5);

        TitanButton btnToggleClimbMechPiston = new TitanButton(oi.leftJoystick, 3);
        btnToggleHopperIntake = new TitanButton(oi.rightJoystick, 3);
        btnToggleHopperExpel = new TitanButton(oi.rightJoystick, 2);

        btnToggleIntake = new TitanButton(oi.getXbox(), 1);
        btnFeederExpel = new TitanButton(oi.getXbox(), 3);
        btnIncreaseShooterSpeed = new TitanButton(oi.getXbox(), OI.BTNNUM_INCREASE_SHOOT_SPEED);
        btnDecreaseShooterSpeed = new TitanButton(oi.getXbox(), OI.BTNNUM_DECREASE_SHOOT_SPEED);


        // MARK - bindings
        btnToggleShifter.whenPressed(new ToggleGearShifter(driveTrain));
        btnToggleIntake.whenPressed(new ToggleIntake(intake));
        btnToggleHopperIntake.whileHeld(new HopperIntake(hopper));
        btnToggleHopperExpel.whileHeld(new HopperExpel(hopper));

        btnFeederExpel.whileHeld(new FeedBall(feeder));

        btnIncreaseShooterSpeed.whenPressed(new InstantCommand(() -> {
            turret.increaseRPMSetpoint();
            turret.setShooterVelocityRPM(turret.getRPMSetpoint());
        }, turret));
        btnDecreaseShooterSpeed.whenPressed(new InstantCommand(() -> {
            turret.decreaseRPMSetpoint();
            turret.setShooterVelocityRPM(turret.getRPMSetpoint());
        }, turret));


        btnToggleColorMechPiston.whenPressed(() -> {
            colorMechPiston.set(!colorMechPiston.get());
        });
        btnToggleClimbMechPiston.whenPressed(() -> {
            climbMechPiston.set(!climbMechPiston.get());
        });




    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
//        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
//                new SimpleMotorFeedforward(kS, kV, kA),
//                kinematics,
//                MAX_VOLTAGE
//        );
//        TrajectoryConfig config = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION).setKinematics(kinematics);
//        Trajectory autoTrajectory = TrajectoryGenerator.generateTrajectory(
//                new Pose2d(0, 0, new Rotation2d(0)), //start
//                List.of(
//                        new Translation2d(1, 1),
//                        new Translation2d(2, 2)
//                ),
//                new Pose2d(3, 3, new Rotation2d(0)), //end
//                config
//        );
//
//        RamseteCommand ramseteCommand = new RamseteCommand(
//                autoTrajectory,
//                driveTrain::getPose,
//                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
//                new SimpleMotorFeedforward(kS, kV, kA),
//                kinematics,
//                driveTrain::getWheelSpeeds,
//                new PIDController(0, 0, 0),
//                new PIDController(0, 0, 0),
//                driveTrain::tankDriveVolts,
//                driveTrain
//        );
//
//        return ramseteCommand.andThen(autonomousCommand);
        return autonomousCommand;
    }
}



