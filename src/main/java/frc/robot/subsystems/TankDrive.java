package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.motors.Encoder;
import frc.robot.motors.TitanFX;
import frc.robot.sensors.TitanGyro;

public class TankDrive extends DriveTrain {

    private TitanFX left;
    private TitanFX right;
    private Gyro gyro;

    private DifferentialDriveOdometry odometry;

    //TankDrive setup

    public TankDrive(TitanFX leftTalonFX, TitanFX rightTalonFX, DifferentialDriveKinematics kinematics) {
        this(leftTalonFX, rightTalonFX, new TitanGyro(new AnalogGyro(12)), kinematics);
        this.odometry = new DifferentialDriveOdometry(getAngle(), new Pose2d());
    }

    public TankDrive(TitanFX leftTalonFX, TitanFX rightTalonFX, Gyro gyro, DifferentialDriveKinematics kinematics) {
        this.left = leftTalonFX;
        this.right = rightTalonFX;
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    //set the speed the motors

    public void periodic(){
        odometry.update(getAngle(), left.getEncoder().getDistance(), right.getEncoder().getDistance());
    }

    public void set(double speed) {
        left.set(speed);
        right.set(speed);
    }

    public void setLeft(double speed) {
        left.set(speed);
    }

    public void setRight(double speed) {
        right.set(speed);
    }

    public void set(double leftTSpeed, double rightTSpeed) {
        left.set(leftTSpeed);
        right.set(rightTSpeed);
    }

    @Override
    public void stop() {

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

    @Override
    public void coast() {
        left.coast();
        right.coast();
    }

    @Override
    public void resetEncoders() {

    }

    @Override
    public Encoder getLeftEncoder() {
        return (Encoder) left.getEncoder();
    }

    @Override
    public Encoder getRightEncoder() {
        return (Encoder) right.getEncoder();
    }

    @Override
    public TitanFX getLeft() {
        return left;
    }

    @Override
    public TitanFX getRight() {
        return right;
    }

    @Override
    public void enableBrownoutProtection() {
        left.enableBrownoutProtection();
        right.enableBrownoutProtection();
    }

    @Override
    public void disableBrownoutProtection() {
        left.disableBrownoutProtection();
        right.disableBrownoutProtection();
    }

    @Override
    public double[] getSpeed() {
        return new double[] { left.getSpeed(), right.getSpeed() };
    }
    public Gyro getGyro() {
        return gyro;
    }

    public boolean hasGyro() {
        return !(gyro == null);
    }

}

