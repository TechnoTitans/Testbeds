package frc.robot.sensors;

import edu.wpi.first.wpilibj.interfaces.Gyro;

@SuppressWarnings("WeakerAccess")

public class TitanGyro implements Gyro {

    protected Gyro m_internalGyro;
    private double angleOffset = 0;

    /**
     * The reason we make a class that accepts a gyro is because the alternative would be to create a central shared
     * gyro in TechnoTitan and reuse that is because there are a lot of issues with sharing such as resetting and shared global
     * state™. And we know that Shared Global State™ is a Very Bad Thing®.
     * @param gyroToAccept - the underlying gyro to use
     */
    public TitanGyro(Gyro gyroToAccept) {
        this.m_internalGyro = gyroToAccept;
    }

    public void resetTo(double angle) {
        this.angleOffset = m_internalGyro.getAngle() - angle;
    }

    @Override
    public void calibrate() {
        m_internalGyro.calibrate();
    }

    @Override
    public void reset() {
        resetTo(0);
    }

    @Override
    public double getAngle() {
//         m_gyro_angle - angleOffset = angle_rst
//         m_gyro_angle - angle_rst = angleOffset
        return m_internalGyro.getAngle() - angleOffset;
    }

    @Override
    public double getRate() {
        return m_internalGyro.getRate();
    }


    @Override
    public void close() throws Exception {
        m_internalGyro.close();
    }
}