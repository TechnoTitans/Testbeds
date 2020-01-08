package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Example_Subsystem extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    /**
     * The Singleton instance of this Example_Subsystem. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static Example_Subsystem INSTANCE = new Example_Subsystem();

    /**
     * Creates a new instance of this Example_Subsystem.
     * This constructor is private since this class is a Singleton. External classes
     * should use the {@link #getInstance()} method to get the instance.
     */
    private Example_Subsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    /**
     * Returns the Singleton instance of this Example_Subsystem. This static method
     * should be used -- {@code Example_Subsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static Example_Subsystem getInstance() {
        return INSTANCE;
    }

}

