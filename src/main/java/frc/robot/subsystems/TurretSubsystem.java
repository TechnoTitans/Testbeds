package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.TitanSRX;
import frc.robot.motor.Encoder;
import frc.robot.motor.TitanVictor;

public class TurretSubsystem extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    /**
     * The Singleton instance of this TurretSubsystem. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
//    private final static TurretSubsystem INSTANCE = new TurretSubsystem(shooter, zMotor);

    /**
     * Creates a new instance of this TurretSubsystem.
     * This constructor is private since this class is a Singleton. External classes
     * should use the {@link #getInstance()} method to get the instance.
     */
    private TitanSRX shooter, zMotor, hood, belt;
    private TitanVictor subShoot;
    private PIDController zMotorPID, hoodPID;
    private DigitalInput beltLimitSwitch;

    private double manualSpeedSetpoint;

    public TurretSubsystem(TitanSRX shooter, TitanVictor subShoot, TitanSRX zMotor, TitanSRX hood, TitanSRX belt, DigitalInput beltLimitSwitch) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        this.shooter = shooter;
        this.subShoot = subShoot;
        this.zMotor = zMotor;
        this.hood = hood;
        this.belt = belt;
        this.beltLimitSwitch = beltLimitSwitch;
        zMotorPID = new PIDController(0, 0, 0);
        hoodPID = new PIDController(0, 0, 0);

    }

    public void setShooter(double speed) {
        shooter.set(speed);
    }

    public void setSubShoot(double speed) {
        subShoot.set(speed);
    }

    public void setZMotor(double speed) {
        zMotor.set(speed);
    }

    public void setHood(double speed) {
        hood.set(speed);
    }

    public void setBelt(double speed) {
        belt.set(speed);
    }

    public Encoder getZMotorEncoder() {
        return zMotor.getEncoder();
    }

    public Encoder getHoodEncoder() {
        return hood.getEncoder();
    }

    public PIDController getZMotorPID() {
        return zMotorPID;
    }

    public PIDController getHoodPID() {
        return hoodPID;
    }

    public DigitalInput getBeltLimitSwitch() {
        return beltLimitSwitch;
    }

    public TitanSRX getShooter(){
        return shooter;
    }

    public void setSpeedSetpoint(double speed) {
        if (speed >= 1) {
            speed = 1;
        } else if (speed <= -1) {
            speed = -1;
        }
        manualSpeedSetpoint = speed;
    }

    public double getSpeedSetpoint() {
        return manualSpeedSetpoint;
    }

    /**
     * Returns the Singleton instance of this TurretSubsystem. This static method
     * should be used -- {@code TurretSubsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
//    public static TurretSubsystem getInstance() {
//        return INSTANCE;
//    }

}

