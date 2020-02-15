/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotMap {
    public static I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard;


    // PCM Numbers
    public static final int GEAR_SHIFT_SOLENOID = 0;
    public static final int INTAKE_SOLENOID = 1;
    public static final int FALCON_COOLING_PORT = 2;

    //  Can IDS

    public static final int COMPRESSOR_ID = 2;

    public static final int LEFT_TALON_FRONT = 4, LEFT_TALON_BACK = 3;
    public static final boolean REVERSED_LF_TALON = true, REVERSED_LB_TALON = true;

    public static final int RIGHT_TALON_FRONT = 5, RIGHT_TALON_BACK = 6;
    public static final boolean REVERSED_RF_TALON = false, REVERSED_RB_TALON = false;

    public static final int INTAKE_MOTOR = 7; // victor

    public static final boolean REVERSED_INTAKE_MOTOR = false;

    public static final int HOPPER_MOTOR = 8; // victor
    public static final boolean REVERSED_HOPPER_MOTOR = false;
    public static final int HOOD = 9; // victor
    public static final boolean REVERSED_HOOD = false;
    public static final int FLYWHEEL1 = 10;
    public static final boolean REVERSED_FLYWHEEL1 = false;
    public static final int FLYWHEEL2 = 11; // victor
    public static final boolean REVERSED_FLYWHEEL2 = false;
    public static final int TURRET_ROTATION = 12;
    public static final boolean REVERSED_TURRET_ROTATION = false;
    public static final int FEEDER_MOTOR = 13; // victor
    public static final boolean REVERSED_FEEDER = false;

    public static final int WINCH_MOTOR = 0;

    // Joysticks
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final int AUX_JOYSTICK_1 = 2;
}
