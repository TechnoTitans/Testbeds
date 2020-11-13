package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.Encoder;
import frc.robot.motor.TitanSRX;
import frc.robot.sensors.TitanGyro;


@SuppressWarnings("ConstantConditions")
public class TankDrive extends SubsystemBase {

	//todo find actual inches_per_pulse
	public static final double DRIVETRAIN_INCHES_PER_PULSE = 100 / 82763f; // inches per pulse for encoder

	private TitanSRX main;
	private Gyro gyro;

	//TankDrive setup

	public TankDrive(TitanSRX mainTalonSRX) {
		this(mainTalonSRX, new TitanGyro(new AnalogGyro(0)));
	}

	public TankDrive(TitanSRX mainTalonSRX, Gyro gyro) {
		this.main = mainTalonSRX;
		this.gyro = gyro;
		resetEncoders();
	}

	public void set(double speed) {
		main.set(speed);
	}

//	public void set(double leftTSpeed, double rightTSpeed) {
//		main.set(leftTSpeed);
//		right.set(rightTSpeed);
//	}

	public void stop() {
		this.set(0);
	}

	//Turning in place

//	public void turnInPlace(boolean ifRight, double speed) {
//		if (ifRight) {
//			main.set(speed);
//			right.set(-speed);
//		} else {
//			main.set(-speed);
//			right.set(speed);
//		}
//	}

	//Other movements

	public void brake() {
		main.brake();
	}


	public void coast() {
		main.coast();
	}


	public void resetEncoders() {
		this.main.getEncoder().reset();
	}


	public Encoder getEncoder() {
		return main.getEncoder();
	}




	public TitanSRX getMotor() {
		return main;
	}




	public void enableBrownoutProtection() {
		main.enableBrownoutProtection();
	}


	public void disableBrownoutProtection() {
		main.disableBrownoutProtection();
	}

	public Gyro getGyro() {
		return gyro;
	}

	public boolean hasGyro() {
		return !(gyro == null);
	}

}

