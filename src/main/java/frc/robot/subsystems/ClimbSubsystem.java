package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class ClimbSubsystem implements Subsystem {

    Solenoid LeftPiston = new Solenoid(1);
    Solenoid RightPiston = new Solenoid(2);
    Spark winchMotor = new Spark(8);


    public void openPistons() {
        LeftPiston.set(true);
        RightPiston.set(true);
    }

    public void closePistons() {
        LeftPiston.set(false);
        RightPiston.set(false);
    }

    public void PullUp() {
        winchMotor.set(-1);
    }

    public void reversePullUp() {
        winchMotor.set(1);
    }

}

