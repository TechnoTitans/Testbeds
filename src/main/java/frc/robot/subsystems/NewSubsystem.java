package frc.robot.subsystems;


import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class NewSubsystem extends Subsystem {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    /**
     * The Singleton instance of this NewSubsystem. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static NewSubsystem INSTANCE = new NewSubsystem();

    /**
     * Creates a new instance of this NewSubsystem.
     * This constructor is private since this class is a Singleton. External classes
     * should use the {@link #getInstance()} method to get the instance.
     */
    private NewSubsystem() {

    }

    /**
     * Returns the Singleton instance of this NewSubsystem. This static method
     * should be used -- {@code NewSubsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static NewSubsystem getInstance() {
        return INSTANCE;
    }

    @Override
    protected void initDefaultCommand() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       e.g. setDefaultCommand(new MyCommand());
    }
}

