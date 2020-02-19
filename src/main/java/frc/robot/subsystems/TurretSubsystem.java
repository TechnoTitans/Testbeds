package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.TitanSRX;
import frc.robot.motor.Encoder;
import frc.robot.motor.TitanVictor;

@SuppressWarnings("JavadocReference")
public class TurretSubsystem extends SubsystemBase {


    public static final double HOOD_PULSES_PER_DEGREE = (187  + 857) / 24.2; // (pulses per degree)
    public static final double ZMOTOR_PULSES_PER_DEGREE = (-5772f) / 45; // (pulses per degree)
    public static final double FLYWHEEL_PULSES_PER_REVOLUTION = (4100 + 40); // (pulses per rev)

    private final double MAX_RPM = 7400; //18730 max rpm / 2.5 gear reduction ratio
    private final double RPM_INCREMENT = 0.1 * this.MAX_RPM;
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
    private TitanSRX shooter, zMotor, hood;
    private TitanVictor subShoot;
    private PIDController zMotorPID, hoodPID, shooterPID;
    private DigitalInput beltLimitSwitch;

    private double manualPercentOutputSetpoint;
    private double rpmSetpoint;
    public TurretSubsystem(TitanSRX shooter, TitanVictor subShoot, TitanSRX zMotor, TitanSRX hood, DigitalInput beltLimitSwitch) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        this.shooter = shooter;
        this.subShoot = subShoot;
        this.zMotor = zMotor;
        this.hood = hood;
        this.beltLimitSwitch = beltLimitSwitch;
        zMotorPID = new PIDController(0, 0, 0);
        hoodPID = new PIDController(0, 0, 0);
        shooterPID = new PIDController(0, 0, 0);
    }

    public void setShooter(double speed) {
        shooter.set(speed);
    }

    public void setSubShoot(double speed) {
        subShoot.set(speed);
    }

    public void setShooterVelocityRPM (double rpm){
        shooter.setVelocityRPM(rpm);
    }

    public void setZMotor(double speed) {
        zMotor.set(speed);
    }

    public void setHood(double speed) {
        hood.set(speed);
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

    public PIDController getShooterPID() { return shooterPID; }

    public DigitalInput getBeltLimitSwitch() {
        return beltLimitSwitch;
    }

    public TitanSRX getShooter(){
        return shooter;
    }

    public void setPercentOutputSetpoint(double speed) {
        if (speed >= 1) {
            speed = 1;
        } else if (speed <= -1) {
            speed = -1;
        }
        if (speed < 0){
            speed = 0;
        }
        manualPercentOutputSetpoint = speed;
    }

    public void setRPMSetpoint(double rpm) {
        this.rpmSetpoint = rpm;
        shooterPID.setSetpoint(getRPMSetpoint());
    }

    public double getRPMSetpoint(){
        return rpmSetpoint;
    }

    public double getPercentOutputSetpoint() {
        return manualPercentOutputSetpoint;
    }

    public void increaseRPMSetpoint() {
        this.rpmSetpoint += this.RPM_INCREMENT;
    }

    public void decreaseRPMSetpoint() {
        this.rpmSetpoint -= this.RPM_INCREMENT;
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

