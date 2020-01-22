package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.motors.Motor;
import frc.robot.motors.TitanFX;


public class ClimbSubsystem implements Subsystem {

    TitanFX motor = new TitanFX(8, false);
    Solenoid leftPiston = new Solenoid(1);
    Solenoid rightPiston = new Solenoid(2);



    public boolean isPistonOpen = false;

    public ClimbSubsystem(TitanFX pullUpMotor){
        this.motor = pullUpMotor;
    }

    public void pullUp() {
        motor.set(1);
    }

    public void openPistons() {
        leftPiston.set(true);
        rightPiston.set(true);
        isPistonOpen = true;
    }

    public void closePistons() {
        leftPiston.set(false);
        rightPiston.set(false);
        isPistonOpen = false;
    }


}

