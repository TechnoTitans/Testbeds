package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.motors.Encoder;
import frc.robot.motors.TitanFX;
import frc.robot.sensors.TitanGyro;

@SuppressWarnings("ConstantConditions")
public class TankDrive extends DriveTrain {


    public static boolean SHIFT_HIGH_TORQUE = true; // todo find actual value
    public static boolean SHIFT_LOW_TORQUE = !SHIFT_HIGH_TORQUE; // for programmers' convenience

    private TitanFX left;
    private TitanFX right;
    private Gyro gyro;
    private Solenoid shifterSolenoid;


    //TankDrive setup

    public TankDrive(TitanFX leftTalonFX, TitanFX rightTalonFX, Solenoid shifterSolenoid) {
        this(leftTalonFX, rightTalonFX, new TitanGyro(new AnalogGyro(12)), shifterSolenoid);
    }

    public TankDrive(TitanFX leftTalonFX, TitanFX rightTalonFX, Gyro gyro, Solenoid shifterSolenoid) {
        this.left = leftTalonFX;
        this.right = rightTalonFX;
        this.shifterSolenoid = shifterSolenoid;
    }

    //set the speed the motors

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

    @Override
    public void coast() {
        left.coast();
        right.coast();
    }

    @Override
    public void resetEncoders() {
        this.left.getEncoder().reset();
        this.right.getEncoder().reset();
    }

    @Override
    public Encoder getLeftEncoder() {
        return left.getEncoder();
    }

    @Override
    public Encoder getRightEncoder() {
        return right.getEncoder();
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
    }

}

