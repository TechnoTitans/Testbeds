package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.motor.TalonSRX;


public class ClimbSubsystem implements Subsystem {

    private TalonSRX Motor;
    Solenoid LeftPiston = new Solenoid(1);
    Solenoid RightPiston = new Solenoid(2);

    public void robotInit() {
        LeftPiston.close();
        RightPiston.close();
        Motor.stop();
    }

    public ClimbSubsystem(TalonSRX pullUpMotor){
        this.Motor = pullUpMotor;
    }

    public void pullUp() {
        Motor.set(1);
    }

    public void reversePullUp() {
        Motor.set(-1);
    }

    public void openPistons() {
        LeftPiston.set(true);
        RightPiston.set(true);
    }

    public void closePistons() {
        LeftPiston.set(false);
        RightPiston.set(false);
    }
}

