package frc.robot;

import frc.robot.subsystems.TurretSubsystem;

public class PIDConstants {

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public final static Gains Shooter_Velocity_Gains = new Gains(0.35, 0.001, 0, (1023.0 / TurretSubsystem.MAX_RPM), 0, 1.00);
    public final static Gains Turret_Position_Gains = new Gains(0.35, 0.001, 0, (1023.0 / TurretSubsystem.MAX_RPM), 0, 1.00);


}
