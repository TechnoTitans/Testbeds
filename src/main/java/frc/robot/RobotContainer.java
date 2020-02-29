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
import frc.robot.commands.*;
import frc.robot.commands.auto.DriveStraightAuto;
import frc.robot.motor.TitanFX;
import frc.robot.motor.TitanSRX;
import frc.robot.motor.TitanVictor;
import frc.robot.sensors.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("FieldCanBeLocal")
public class RobotContainer {

    private final Compressor compressor;
    private final TitanVictor feederMotor;
    private final FeederSubsystem feeder;

    public final QuadEncoder zMotorEncoder;
    public final QuadEncoder hoodMotorEncoder;
    public final QuadEncoder shootMotorEncoder;
    public final QuadEncoder leftFrontMotorEncoder;
    public final QuadEncoder rightFrontMotorEncoder;
    public final QuadEncoder rightBackMotorEncoder;
    public final QuadEncoder leftBackMotorEncoder;

    public final Vision vision;

    // MARK - Solenoids

    public final Solenoid shifterSolenoid;
    public final Solenoid intakeSolenoid;
    private final Solenoid climbMechPiston;
    private final Solenoid colorMechPiston;
    public final Solenoid titanFXCoolingPiston;


    public final TitanSRX shootMotor;
    private final TitanVictor subShootMotor;
    public final TitanSRX zMotor;
    public final TitanSRX hoodMotor;
    private final TitanSRX spinningMotor;
    public final TitanSRX intakeMotor;
    private final TitanVictor hopperMotor;
    public final TitanSRX climbMotor;

    private final LimitSwitch leftTurretLS;
    private final LimitSwitch rightTurretLS;
    public final LimitSwitch hoodBottomLS;


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
    public ClimbSubsystem climb;

    // MARK - Command declarations
    public DriveTrainCommand driveTrainCommand;
    public ToggleGearShifter toggleGearShifterCommand;
    public IntakeTeleop intakeTeleopCommand;
    public TurretTeleop turretTeleopCommand;
    public TurretAutonomous turretAutoCommand;


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
    private TitanButton btnToggleSolenoid;
    private TitanButton btnTurretAutoAim;
    private TitanButton btnTurretManual;
    private TitanButton btnToggleColorMechPiston;
    private TitanButton btnReleaseClimbMechPiston;

    private Trigger climbPositiveTrigger;
	private Trigger climbNegativeTrigger;


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

    public static DriverCamera driverCamera;


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
        hoodBottomLS = new LimitSwitch(RobotMap.HOOD_BOTTOM_LS, RobotMap.HOOD_BOTTOM_LS_INVERTED);
        turret = new TurretSubsystem(shootMotor, subShootMotor, zMotor, hoodMotor, leftTurretLS, rightTurretLS, hoodBottomLS);
        spinningMotor = new TitanSRX(RobotMap.COLOR_WHEEL_MOTOR, RobotMap.REVERSED_COLOR_WHEEL);
//        colorSensor = new ColorSensorV3(RobotMap.COLOR_SENSOR_PORT);
//        controlPanel = new ControlPanelSubsystem(spinningMotor, colorSensor);

        leftFrontMotorFX = new TitanFX(RobotMap.LEFT_TALON_FRONT, RobotMap.REVERSED_LF_TALON);
        leftBackMotorFX = new TitanFX(RobotMap.LEFT_TALON_BACK, RobotMap.REVERSED_LB_TALON);
        rightFrontMotorFX = new TitanFX(RobotMap.RIGHT_TALON_FRONT, RobotMap.REVERSED_RF_TALON);
        rightBackMotorFX = new TitanFX(RobotMap.RIGHT_TALON_BACK, RobotMap.REVERSED_RB_TALON);

        leftFrontMotorEncoder = new QuadEncoder(leftFrontMotorFX, TankDrive.DRIVETRAIN_INCHES_PER_PULSE, false);
        leftFrontMotorFX.setEncoder(leftFrontMotorEncoder);
        rightFrontMotorEncoder = new QuadEncoder(rightFrontMotorFX, TankDrive.DRIVETRAIN_INCHES_PER_PULSE, false);
        rightFrontMotorFX.setEncoder(rightFrontMotorEncoder);
        rightBackMotorEncoder = new QuadEncoder(rightBackMotorFX, TankDrive.DRIVETRAIN_INCHES_PER_PULSE, false);
        rightBackMotorFX.setEncoder(rightBackMotorEncoder);
        leftBackMotorEncoder = new QuadEncoder(leftBackMotorFX, TankDrive.DRIVETRAIN_INCHES_PER_PULSE, false);
        leftBackMotorFX.setEncoder(leftBackMotorEncoder);


        leftBackMotorFX.follow(leftFrontMotorFX);
        rightBackMotorFX.follow(rightFrontMotorFX);


        kinematics = new DifferentialDriveKinematics(DRIVE_WIDTH);

        shifterSolenoid = new Solenoid(RobotMap.COMPRESSOR_ID, RobotMap.GEAR_SHIFT_SOLENOID);
        titanFXCoolingPiston = new Solenoid(RobotMap.COMPRESSOR_ID, RobotMap.FALCON_COOLING_PORT);

        driveTrain = new TankDrive(leftFrontMotorFX, rightFrontMotorFX, shifterSolenoid, titanFXCoolingPiston, compressor);


        intakeMotor = new TitanSRX(RobotMap.INTAKE_MOTOR, RobotMap.REVERSED_INTAKE_MOTOR);
        intakeSolenoid = new Solenoid(RobotMap.COMPRESSOR_ID, RobotMap.INTAKE_SOLENOID);
        intake = new IntakeSubsystem(intakeMotor, intakeSolenoid);

        hopperMotor = new TitanVictor(RobotMap.HOPPER_MOTOR, RobotMap.REVERSED_HOPPER_MOTOR);
        hopper = new HopperSubsystem(hopperMotor);

        feederMotor = new TitanVictor(RobotMap.FEEDER_MOTOR, RobotMap.REVERSED_FEEDER);
        feeder = new FeederSubsystem(feederMotor);

//        vision = null; // this is bad. only for testing. don't do this.!!!
        vision = new Vision();

        // todo move and put into actual subsystem
        climbMechPiston = new Solenoid(RobotMap.COMPRESSOR_ID, RobotMap.CLIMB_MECH_PISTON);
        climbMotor = new TitanSRX(RobotMap.WINCH_MOTOR, RobotMap.WINCH_MOTOR_REVERSED);
        climb = new ClimbSubsystem(climbMotor, climbMechPiston);

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
        shootMotor.configPeakOutputForward(PIDConstants.Shooter_Velocity_Gains.kPeakOutput, PIDConstants.kTimeoutMs);
        shootMotor.configPeakOutputReverse(-PIDConstants.Shooter_Velocity_Gains.kPeakOutput, PIDConstants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        shootMotor.config_kF(PIDConstants.kSlotIdx, PIDConstants.Shooter_Velocity_Gains.kF, PIDConstants.kTimeoutMs);
        shootMotor.config_kP(PIDConstants.kSlotIdx, PIDConstants.Shooter_Velocity_Gains.kP, PIDConstants.kTimeoutMs);
        shootMotor.config_kI(PIDConstants.kSlotIdx, PIDConstants.Shooter_Velocity_Gains.kI, PIDConstants.kTimeoutMs);
        shootMotor.config_kD(PIDConstants.kSlotIdx, PIDConstants.Shooter_Velocity_Gains.kD, PIDConstants.kTimeoutMs);

        zMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                PIDConstants.kPIDLoopIdx,
                PIDConstants.kTimeoutMs);

        zMotor.configNominalOutputForward(0, PIDConstants.kTimeoutMs);
        zMotor.configNominalOutputReverse(0, PIDConstants.kTimeoutMs);
        zMotor.configPeakOutputForward(PIDConstants.Turret_ZMotor_Gains.kPeakOutput, PIDConstants.kTimeoutMs);
        zMotor.configPeakOutputReverse(-PIDConstants.Turret_ZMotor_Gains.kPeakOutput, PIDConstants.kTimeoutMs);

        zMotor.config_kF(PIDConstants.kSlotIdx, PIDConstants.Turret_ZMotor_Gains.kF, PIDConstants.kTimeoutMs);
        zMotor.config_kP(PIDConstants.kSlotIdx, PIDConstants.Turret_ZMotor_Gains.kP, PIDConstants.kTimeoutMs);
        zMotor.config_kI(PIDConstants.kSlotIdx, PIDConstants.Turret_ZMotor_Gains.kI, PIDConstants.kTimeoutMs);
        zMotor.config_kD(PIDConstants.kSlotIdx, PIDConstants.Turret_ZMotor_Gains.kD, PIDConstants.kTimeoutMs);

        hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                PIDConstants.kPIDLoopIdx,
                PIDConstants.kTimeoutMs);

        hoodMotor.configNominalOutputForward(0, PIDConstants.kTimeoutMs);
        hoodMotor.configNominalOutputReverse(0, PIDConstants.kTimeoutMs);
        hoodMotor.configPeakOutputForward(PIDConstants.Turret_Hood_Gains.kPeakOutput, PIDConstants.kTimeoutMs);
        hoodMotor.configPeakOutputReverse(-PIDConstants.Turret_Hood_Gains.kPeakOutput, PIDConstants.kTimeoutMs);

        hoodMotor.config_kF(PIDConstants.kSlotIdx, PIDConstants.Turret_Hood_Gains.kF, PIDConstants.kTimeoutMs);
        hoodMotor.config_kP(PIDConstants.kSlotIdx, PIDConstants.Turret_Hood_Gains.kP, PIDConstants.kTimeoutMs);
        hoodMotor.config_kI(PIDConstants.kSlotIdx, PIDConstants.Turret_Hood_Gains.kI, PIDConstants.kTimeoutMs);
        hoodMotor.config_kD(PIDConstants.kSlotIdx, PIDConstants.Turret_Hood_Gains.kD, PIDConstants.kTimeoutMs);

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

        // MARK - Button Definitions
        btnToggleShifter = new TitanButton(oi.leftJoystick, 4);
        btnToggleSolenoid = new TitanButton(oi.leftJoystick, 2);

//        btnToggleSolenoid.whenPressed(new InstantCommand(() -> {
//            if (titanFXCoolingPiston.get()){
//                titanFXCoolingPiston.set(false);
//            } else {
//                titanFXCoolingPiston.set(true);
//            }
//        }, driveTrain));
        btnToggleColorMechPiston = new TitanButton(oi.leftJoystick, 5);
        btnReleaseClimbMechPiston = new TitanButton(oi.leftJoystick, 3);
        btnToggleHopperIntake = new TitanButton(oi.rightJoystick, 3);
        btnToggleHopperExpel = new TitanButton(oi.rightJoystick, 2);

        btnToggleIntake = new TitanButton(oi.getXbox(), 1);
        btnFeederExpel = new TitanButton(oi.getXbox(), 3);
        btnIncreaseShooterSpeed = new TitanButton(oi.getXbox(), OI.XBOX_BUMPER_RIGHT);
        btnDecreaseShooterSpeed = new TitanButton(oi.getXbox(), OI.XBOX_BUMPER_LEFT);
        btnTurretAutoAim = new TitanButton(oi.getXbox(), OI.XBOX_BTN_START);
        btnTurretManual = new TitanButton(oi.getXbox(), OI.XBOX_BTN_SELECT);

        // MARK - command initialization
        driveTrainCommand = new DriveTrainCommand(oi::getLeftJoyY, oi::getRightJoyY, driveTrain, true);
        toggleGearShifterCommand = new ToggleGearShifter(driveTrain);

        intakeTeleopCommand = new IntakeTeleop(oi::getXboxLeftY, intake, btnToggleIntake);
        turretTeleopCommand = new TurretTeleop(oi::getXboxRightX, oi::getXboxRightY, turret, true);
        turretAutoCommand = new TurretAutonomous(vision, turret);


        autonomousCommand = new DriveStraightAuto(driveTrain, 3f * 12, 0.1);
//        autonomousCommand = new DoNothingAuto();
        // Configure the button bindings
        configureButtonBindings();

        driverCamera = new DriverCamera();

        CommandScheduler.getInstance().registerSubsystem(climb);
        CommandScheduler.getInstance().registerSubsystem(turret);
        CommandScheduler.getInstance().registerSubsystem(driveTrain);
        CommandScheduler.getInstance().registerSubsystem(controlPanel);
        CommandScheduler.getInstance().registerSubsystem(feeder);
        CommandScheduler.getInstance().registerSubsystem(intake);

    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
     */
    private void configureButtonBindings() {

        // MARK - bindings

//        btnToggleShifter.whenPressed(new ToggleGearShifter(driveTrain));
        btnToggleHopperIntake.whileHeld(new HopperIntake(hopper));
        btnToggleHopperExpel.whileHeld(new HopperExpel(hopper));

        btnFeederExpel.whileHeld(new FeedBall(feeder));

        btnIncreaseShooterSpeed.whenPressed(new InstantCommand(() -> {
            turret.increaseRPMSetpoint();
        }, turret));
        btnDecreaseShooterSpeed.whenPressed(new InstantCommand(() -> {
            turret.decreaseRPMSetpoint();
        }, turret));


        btnToggleColorMechPiston.whenPressed(() -> {
            colorMechPiston.set(!colorMechPiston.get());
        });

        btnReleaseClimbMechPiston.whenPressed(() -> {
            climb.releaseMech();
        });
        // pov (d-pad):
		// a: 0deg, b: -90deg, c: 90deg, d: 180
        //   _ a _
		//   b _ c
		//   _ d _
		climbNegativeTrigger = new Trigger(() -> oi.getXbox().getPOV() == 0).whileActiveContinuous(new MoveWinchMotor(climb, MoveWinchMotor.Direction.NEGATIVE));
		climbPositiveTrigger = new Trigger(() -> oi.getXbox().getPOV() == 180).whileActiveContinuous(new MoveWinchMotor(climb, MoveWinchMotor.Direction.POSITIVE));


//		btnTurretAutoAim.whenPressed(new InstantCommand(() -> {
//            CommandScheduler.getInstance().cancel(turretTeleopCommand);
//            CommandScheduler.getInstance().schedule(turretAutoCommand);
//        }));

        btnTurretManual.whenPressed(new InstantCommand(() -> {
            CommandScheduler.getInstance().cancel(turretAutoCommand);
            CommandScheduler.getInstance().schedule(turretTeleopCommand);
        }));

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



