package frc.robot.sensors;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.motors.Encoder;
import frc.robot.motors.Motor;
import frc.robot.motors.TitanFX;

/**
 * Encoder class. Used to measure how far the robot traveled
 */

public class QuadEncoderFX implements Encoder {

	private TitanFX talonFX;
	public static final double PULSES_PER_ROTATION = 4096;
	private double inchesPerPulse; // configure

	public QuadEncoderFX(TitanFX talonFX, double inchesPerPulse, boolean reversed) {
		this.talonFX = talonFX;
		this.inchesPerPulse = inchesPerPulse;
		// this.talonSRX.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		this.talonFX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		talonFX.setSensorPhase(reversed);
	}

	/**
	 * The total distance that the motor has traveled
	 * 
	 * @return total distance
	 */
	@Override
	public double getDistance() {
		return talonFX.getSelectedSensorPosition(0) * inchesPerPulse;
	}

	/**
	 * Gets speed of the TalonSRX in RPM
	 */
	// speed = enc counts / 100 ms
	// (speed * 60 secs)
	// --------------------------------------
	// 4096 encoder counts * 100 milliseconds
	@Override
	public double getSpeed() {
		return (talonFX.getSelectedSensorVelocity(0) * 60) / (PULSES_PER_ROTATION * 0.1);
	}

	@Override
	public double getSpeedInches() {
		return talonFX.getSelectedSensorVelocity(0) * 10 * inchesPerPulse;
	}

	@Override
	public void reset() {
		talonFX.setSelectedSensorPosition(0, 0, 0);
	}

	@Override
	public void resetToRaw(int position) {
		talonFX.setSelectedSensorPosition(position, 0, 0);
	}

	@Override
	public void resetTo(double position) {
		talonFX.setSelectedSensorPosition((int) (position / inchesPerPulse), 0, 0);
	}

	public double getRawPosition() {
		return talonFX.getSelectedSensorPosition(0);
	}

	@Override
	public double getInchesPerPulse() {
		return inchesPerPulse;
	}

	public Motor getTalon() {
		return talonFX;
	}
}
