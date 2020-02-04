package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.TitanSRX;

public class HopperSubsystem extends SubsystemBase {

    private TitanSRX hopperMotor;

    private static final double INTAKE_SPEED = 1;
    private static final double EXPEL_SPEED = -1;

    public HopperSubsystem(TitanSRX hopperMotor) {
        this.hopperMotor = hopperMotor;
    }
    
    public void stop() { hopperMotor.set(0); }
    public void intake() { hopperMotor.set(INTAKE_SPEED); }
    public void expel() { hopperMotor.set(EXPEL_SPEED); }

    public void setSpeed(double speed) {
        hopperMotor.set(speed);
    }


}
