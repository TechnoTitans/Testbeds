package frc.robot.utils;

public enum TurretPreset {
//    TEST(10, 100),

	OFF(0, 3),
	WALL(2220, 0),
	//    LINE(2960, -568), //-597
	LINE(3108, -568), //-597
	TRENCH(4070, -877); //-906
//    OFF(0, 5),
//    WALL(0, 0),
//    LINE(0, -568), //-597
//    TRENCH(0, -877); //-906
//    ZERO(0,200); // TODO: TEST THIS BEFORE next match just so it zeros the hood angle

	public final double flywheelRPMPreset;
	public final int desiredHoodEncoderPositionPreset;

	TurretPreset(double flywheelRPMPreset, int desiredHoodEncoderPositionPreset) {
		this.flywheelRPMPreset = flywheelRPMPreset;
		this.desiredHoodEncoderPositionPreset = desiredHoodEncoderPositionPreset;
	}

}
