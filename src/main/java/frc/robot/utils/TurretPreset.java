package frc.robot.utils;

public enum TurretPreset {
    OFF(0, 0),
    WALL(2220, 0), //-29
    LINE(2960, -568), //-597
    TRENCH(4070, -877); //-906

    public final double flywheelRPMPreset;
    public final int desiredHoodEncoderPositionPreset;

    TurretPreset(double flywheelRPMPreset, int desiredHoodEncoderPositionPreset) {
        this.flywheelRPMPreset = flywheelRPMPreset;
        this.desiredHoodEncoderPositionPreset = desiredHoodEncoderPositionPreset;
    }

}
