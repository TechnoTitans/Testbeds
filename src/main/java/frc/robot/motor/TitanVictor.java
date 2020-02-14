package frc.robot.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TitanVictor extends com.ctre.phoenix.motorcontrol.can.VictorSPX implements Motor{

    private Encoder encoder;
    private static final int TIMEOUT_MS = 30;
    Gyro gyro;
    public static final int CURRENT_LIMIT = 41;
    public static final int CURRENT_LIMIT_THRESHOLD = 41;
    public static final int LIMIT_TIMEOUT = 200; //ms

    private TitanVictor brownoutFollower = null;
    private boolean brownout = false;

    /**
     * Constructor
     *
     *
     */
    public TitanVictor(int channel, boolean reversed) {
        super(channel);
        super.setInverted(reversed);
    }

    @Override
    public void set(double speed) {
        if (speed > 1) speed = 1;
        if (speed < -1) speed = -1;
        super.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public double getPercentSpeed() {
        return super.getMotorOutputPercent();
    }

    @Override
    public double getSpeed() {
        if (!hasEncoder())
            return 0;
        return encoder.getSpeed();
    }

    @Override
    public void stop() {
        set(0);
    }

    @Override
    public void brake() {
        this.set(0);
        super.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void coast() {
        this.set(0);
        super.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public boolean hasEncoder() {
        return !(this.encoder == null);
    }

    @Override
    public Encoder getEncoder() {
        return encoder;
    }

    public void setEncoder(Encoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public int getChannel() {
        return super.getDeviceID();
    }

    @Override
    public boolean isReversed() {
        return super.getInverted();
    }

    @Override
    public double getCurrent() {
        return -999;
    }
    // Apparently getStatorCurrent() only works for Talons or something, idk how to fix this

    public double getError() {
        return super.getClosedLoopError(0);
    }

    public void enableBrownoutProtection() {
        if (brownoutFollower != null) {
            brownoutFollower.coast();
        }
        brownout = true;
    }

    public void disableBrownoutProtection() {
        if (brownoutFollower != null && brownout) {
            brownoutFollower.setNeutralMode(NeutralMode.Brake);
            brownoutFollower.set(ControlMode.Follower, getChannel());
        }
        brownout = false;
    }

    public void configPID(double P, double D, double F) {
        configPID(P, 0, D, F, 0);
    }
    public void configPID(double P, double I, double D, double F, int iZone) {
        // If these ever need to be nonzero, we can make them parameters instead
        final int pidSlot = 0, profileSlot = 0;

        configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pidSlot, TIMEOUT_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT_MS);
        setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT_MS);

        configNominalOutputForward(0, TIMEOUT_MS);
        configNominalOutputReverse(0, TIMEOUT_MS);
        configPeakOutputForward(1, TIMEOUT_MS);
        configPeakOutputReverse(-1, TIMEOUT_MS);

        // MARK - PID stuff
        selectProfileSlot(profileSlot, pidSlot);
        config_kF(profileSlot, F, TIMEOUT_MS);
        config_kP(profileSlot, P, TIMEOUT_MS);
        config_kI(profileSlot, I, TIMEOUT_MS);
        config_kD(profileSlot, D, TIMEOUT_MS);
        config_IntegralZone(profileSlot, iZone, TIMEOUT_MS);
    }

    private void setStatusFramePeriod(StatusFrameEnhanced status_10_motionMagic, int i, int timeoutMs) {
    }

    public void postEstimatedKf(String name) {
        double speed = getSelectedSensorVelocity(0);
        if (Math.abs(speed) > 10) {
            SmartDashboard.putNumber("Estimated Kf for " + name, getPercentSpeed() / speed * 1023);
        }
    }
}
