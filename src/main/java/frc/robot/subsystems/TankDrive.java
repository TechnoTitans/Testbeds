package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.Encoder;
import frc.robot.motor.TitanSRX;
import frc.robot.sensors.TitanGyro;


@SuppressWarnings("ConstantConditions")
public class TankDrive extends SubsystemBase {

	private TitanSRX left;
	private TitanSRX right;
	private Gyro gyro;

	//TankDrive setup

	public TankDrive(TitanSRX leftTalonSRX, TitanSRX rightTalonSRX) {
		this(leftTalonSRX, rightTalonSRX, new TitanGyro(new AnalogGyro(0)));
	}

	public TankDrive(TitanSRX leftTalonFX, TitanSRX rightTalonFX, Gyro gyro) {
		this.left = leftTalonFX;
		this.right = rightTalonFX;
		this.gyro = gyro;
		resetEncoders();
	}

	public void set(double speed) {
		left.set(speed);
		right.set(speed);
	}

	public void set(double leftTSpeed, double rightTSpeed) {
		left.set(leftTSpeed);
		right.set(rightTSpeed);
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


	public TitanSRX getLeft() {
		return left;
	}


	public TitanSRX getRight() {
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

	public Gyro getGyro() {
		return gyro;
	}

	public boolean hasGyro() {
		return !(gyro == null);
	}

}

