package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.sensors.TitanGyro;

import java.util.HashMap;
import java.util.Map;

public class DriveTrain {

    private TalonFX left;
    private TalonFX right;
    private Gyro gyro;

    public DriveTrain(TalonFX leftFTalonFX, TalonFX rightFTalonFX) {
        this(leftFTalonFX, rightFTalonFX, new TitanGyro(new AnalogGyro(12)));
    }

    public DriveTrain(TalonFX leftFTalonFX, TalonFX rightFTalonFX, Gyro gyro) {
        this.left = leftFTalonFX;
        this.right = rightFTalonFX;
    }

    public void set(double speed) {
        left.set(speed);
        right.set(speed);
    }

    private static final Map<Object, LiveWindow.Component> components = new HashMap<>();
    private boolean m_channelAllocated;

    public AnalogGyro(int channel) {
        this(new AnalogInput(channel));
        m_channelAllocated = true;
        addChild(m_analog);
    }

    public static synchronized void addChild(Sendable parent, Object child) {
        LiveWindow.Component component = components.get(child);
        if (component == null) {
            component = new LiveWindow.Component(null, parent);
            components.put(child, component);
        } else {
            component.m_parent = parent;
        }
        component.m_telemetryEnabled = false;
    }


}

