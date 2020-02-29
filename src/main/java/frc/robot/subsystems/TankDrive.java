package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.sensors.TitanGyro;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motor.Encoder;
import frc.robot.motor.TitanFX;



@SuppressWarnings("ConstantConditions")
public class TankDrive extends SubsystemBase {

    public static final double DRIVETRAIN_INCHES_PER_PULSE = 100 / 82763f; // inches per pulse
    public static final double MAX_MOTOR_TEMP = 50;


    private final SpeedController leftSpeedController;
    private final SpeedController rightSpeedController;
//    private final DifferentialDrive drive;


    public static boolean SHIFT_HIGH_TORQUE = true; // todo find actual value
    public static boolean SHIFT_LOW_TORQUE = !SHIFT_HIGH_TORQUE; // for programmers' convenience

    private TitanFX left;
    private TitanFX right;
    private Gyro gyro;
    private Solenoid shifterSolenoid;
    private Solenoid coolingSolenoid;
    private Compressor compressor;


    private DifferentialDriveOdometry odometry;

    //TankDrive setup

    private PIDController drivePID;
    //TankDrive setup

    public TankDrive(TitanFX leftTalonFX, TitanFX rightTalonFX, Solenoid shifterSolenoid, Solenoid coolingSolenoid, Compressor compressor) {
        this(leftTalonFX, rightTalonFX, new TitanGyro(new AnalogGyro(0)), shifterSolenoid, coolingSolenoid, compressor);
    }

    public TankDrive(TitanFX leftTalonFX, TitanFX rightTalonFX, Gyro gyro, Solenoid shifterSolenoid, Solenoid coolingSolenoid, Compressor compressor) {
        this.left = leftTalonFX;
        this.right = rightTalonFX;
        this.gyro = gyro;
        this.shifterSolenoid = shifterSolenoid;
        this.drivePID = new PIDController(0, 0,0);
//        this.odometry = new DifferentialDriveOdometry(getAngle(), new Pose2d());
        this.leftSpeedController = leftTalonFX;
        this.rightSpeedController = rightTalonFX;
//        this.drive = new DifferentialDrive(leftSpeedController, rightSpeedController);
        this.coolingSolenoid = coolingSolenoid;
        this.compressor = compressor;
        resetEncoders();
    }

    //Odometry stuff

//    public Rotation2d getAngle(){
//        return Rotation2d.fromDegrees(getHeading());
//    }

//    public Pose2d getPose(){
//        return odometry.getPoseMeters();
//    }
//
////    public void periodic(){
////        odometry.update(getAngle(), left.getEncoder().getDistance(), right.getEncoder().getDistance());
////    }
//
//    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
//        return new DifferentialDriveWheelSpeeds(left.getEncoder().getSpeed(), right.getEncoder().getSpeed());
//    }
//    public void resetOdometry(Pose2d pose){
//        resetEncoders();
////        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
//    }
//
//    public void tankDriveVolts(double leftVolts, double rightVolts){
//        leftSpeedController.setVoltage(leftVolts);
//        rightSpeedController.setVoltage(rightVolts);
//    }
//
//    public void setMaxOutput(double maxOutput){
//        drive.setMaxOutput(maxOutput);
//    }

//set the speed the motors

    public PIDController getPID(){
        return drivePID;
    }

    public void set(double speed) {
        left.set(speed);
        right.set(speed);
//        drive.tankDrive(speed, speed);
    }

    public void set(double leftTSpeed, double rightTSpeed) {
        left.set(leftTSpeed);
        right.set(rightTSpeed);
//        drive.tankDrive(leftTSpeed, rightTSpeed);
    }

    public void stop() {
        this.set(0);
    }

    //Turning in place

    public void turnInPlace(boolean ifRight, double speed) {
        if (ifRight) {
            left.set(speed);
            right.set(-speed);
        } else {
            left.set(-speed);
            right.set(speed);
        }
    }

    //Other movements

    public void brake() {
        left.brake();
        right.brake();
    }

    
    public void coast() {
        left.coast();
        right.coast();
    }

    
    public void resetEncoders() {
        this.left.getEncoder().reset();
        this.right.getEncoder().reset();
    }

    
    public Encoder getLeftEncoder() {
        return left.getEncoder();
    }

    
    public Encoder getRightEncoder() {
        return right.getEncoder();
    }

    
    public TitanFX getLeft() {
        return left;
    }

    
    public TitanFX getRight() {
        return right;
    }

    
    public void enableBrownoutProtection() {
        left.enableBrownoutProtection();
        right.enableBrownoutProtection();
    }

    
    public void disableBrownoutProtection() {
        left.disableBrownoutProtection();
        right.disableBrownoutProtection();
    }

    
    public double[] getSpeed() {
        return new double[] { left.getSpeed(), right.getSpeed() };
    }

//    public double getHeading() {
//        return Math.IEEEremainder(gyro.getAngle(), 360); //assuming gyro is not reversed
//    }

    public Gyro getGyro() {
        return gyro;
    }

    public boolean hasGyro() {
        return !(gyro == null);
    }

    //Cooling
    public boolean getCoolingEnabled() {
        return this.coolingSolenoid.get();
    }

    public void setCooling(boolean on) {
        this.coolingSolenoid.set(on);
    }

    public void toggleCooling() {
        boolean currentStatus = this.getCoolingEnabled();
        this.setCooling(!currentStatus);
    }

    public boolean getPressureSwitchValue() {
        return this.compressor.getPressureSwitchValue();
    }

    // MARK - Shifter
    public boolean getShifterEnabled() {
        return this.shifterSolenoid.get();
    }

    public void setShifter(boolean pistonDeployed) {
        this.shifterSolenoid.set(pistonDeployed);
    }

    public void toggleShifter() {
        boolean currentStatus = this.getShifterEnabled();
        this.setShifter(!currentStatus);
//        this.setShifter(true);
    }

}

