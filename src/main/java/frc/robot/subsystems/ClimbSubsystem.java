package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.motor.TitanFX;
import frc.robot.motor.TitanSRX;


public class ClimbSubsystem implements Subsystem {

    TitanSRX motor;
    Solenoid climbSolenoid;

    public ClimbSubsystem(TitanSRX pullUpMotor, Solenoid climbSolenoid){
        this.motor = pullUpMotor;
        this.climbSolenoid = climbSolenoid;
    }

    public void stopMotor() {
        motor.set(0);
    }

    public void pullUp() {
        motor.set(1);
    }

    public void releaseMech() {
        climbSolenoid.set(true);
    }


}

