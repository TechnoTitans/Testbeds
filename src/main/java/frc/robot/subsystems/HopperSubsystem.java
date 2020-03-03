package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.TitanSRX;
import frc.robot.motor.TitanVictor;

public class HopperSubsystem extends SubsystemBase {

    private TitanVictor hopperMotor;

    private static final double INTAKE_SPEED = .25;
    private static final double EXPEL_SPEED = -.25;

    public HopperSubsystem(TitanVictor hopperMotor) {
        this.hopperMotor = hopperMotor;
    }
    
    public void stop() { hopperMotor.set(0); }
    public void intake() { hopperMotor.set(INTAKE_SPEED); }
    public void expel() { hopperMotor.set(EXPEL_SPEED); }

    public void setSpeed(double speed) {
        hopperMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Hopper Runnning", this.hopperMotor.getSelectedSensorVelocity() > 0);
    }
}
