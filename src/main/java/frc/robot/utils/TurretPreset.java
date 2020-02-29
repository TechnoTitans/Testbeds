package frc.robot.utils;

public enum TurretPreset {
    OFF(0, 0),
    WALL(2220, -29),
    LINE(2960, -597),
    TRENCH(4070, -906);

    public final double flywheelRPMPreset;
    public final int desiredHoodEncoderPositionPreset;

    TurretPreset(double flywheelRPMPreset, int desiredHoodEncoderPositionPreset) {
        this.flywheelRPMPreset = flywheelRPMPreset;
        this.desiredHoodEncoderPositionPreset = desiredHoodEncoderPositionPreset;
    }

}
