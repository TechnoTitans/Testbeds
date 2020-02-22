package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.motor.TitanSRX;
import frc.robot.motor.Encoder;
import frc.robot.motor.TitanVictor;
import frc.robot.sensors.LimitSwitch;

@SuppressWarnings("JavadocReference")
public class TurretSubsystem extends SubsystemBase {


    public static final double HOOD_PULSES_PER_DEGREE = (187  + 857) / 24.2; // (pulses per degree)
    public static final double ZMOTOR_PULSES_PER_DEGREE = (-5772f) / 45; // (pulses per degree)
    public static final double FLYWHEEL_PULSES_PER_REVOLUTION = (4100 + 40); // (pulses per rev)
    public static final double MAX_RPM = 7400; //18730 max rpm / 2.5 gear reduction ratio
    public static final double RPM_INCREMENT = 0.1 * TurretSubsystem.MAX_RPM;

    private final LimitSwitch leftTurretLS;
    private final LimitSwitch rightTurretLS;
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

    private double manualPercentOutputSetpoint;
    private double rpmSetpoint;
    public TurretSubsystem(TitanSRX shooter, TitanVictor subShoot, TitanSRX zMotor, TitanSRX hood, LimitSwitch leftTurretLS, LimitSwitch rightTurretLS) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        this.shooter = shooter;
        this.subShoot = subShoot;
        this.zMotor = zMotor;
        this.hood = hood;
        this.leftTurretLS = leftTurretLS;
        this.rightTurretLS  = rightTurretLS;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("[Turret] Left Turret LS", this.leftTurretLS.isPressed());
        SmartDashboard.putBoolean("[Turret] Right Turret LS", this.rightTurretLS.isPressed());
        SmartDashboard.putNumber("[Turret] Asimuth Velocity", this.zMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("[Turret] Hood Position", this.hood.getSelectedSensorPosition());
    }

    public void setShooter(double speed) {
        shooter.set(speed);
    }

    // "subshoot" motor is just a victor, but that victor is a follower so you would never
    // set it individually
    @Deprecated(forRemoval = true)
    public void setSubShoot(double speed) {
        subShoot.set(speed);
    }

    public void setShooterVelocityRPM(double rpm){
        shooter.setVelocityRPM(rpm);
    }

    public void setZMotor(double speed) {
        if (leftTurretLS.isPressed()) {
            speed = MathUtil.clamp(speed, 0, 1); // assuming +1 is right direction
        } else if (rightTurretLS.isPressed()) {
            speed = MathUtil.clamp(speed, -1, 0); // assuming -1 is leftwards
        }
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
        this.setShooterVelocityRPM(rpm);
    }

    public double getRPMSetpoint(){
        return rpmSetpoint;
    }

    public double getPercentOutputSetpoint() {
        return manualPercentOutputSetpoint;
    }

    public void increaseRPMSetpoint() {
        this.rpmSetpoint += this.RPM_INCREMENT;
        this.setRPMSetpoint(this.rpmSetpoint);
    }

    public void decreaseRPMSetpoint() {
        this.rpmSetpoint -= this.RPM_INCREMENT;
        this.setRPMSetpoint(this.rpmSetpoint);
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

