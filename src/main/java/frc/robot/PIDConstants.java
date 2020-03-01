package frc.robot;

import frc.robot.subsystems.TurretSubsystem;

public class PIDConstants {

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public final static Gains Shooter_Velocity_Gains = new Gains(0.03125, 0.0002, 0, 0.75 * (1023.0 / TurretSubsystem.MAX_RPM), 0, 1.00);
    public final static Gains Turret_ZMotor_Gains = new Gains(0.15, 0.00025, 0, 0.0, (int) Math.round(0 * TurretSubsystem.ZMOTOR_PULSES_PER_DEGREE), 0.3);
//    public final static Gains Turret_Hood_Gains = new Gains(0.5, 0.00000, 0, 0.0, (int) Math.round(0 * TurretSubsystem.HOOD_PULSES_PER_DEGREE), 0.3);
    public final static Gains Turret_Hood_Gains = new Gains(12.0, 0.00080, 0, 0.0, (int) Math.round(0 * TurretSubsystem.HOOD_PULSES_PER_DEGREE), 0.3);
}
