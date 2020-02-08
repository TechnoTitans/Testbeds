package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.TitanButton;
import frc.robot.subsystems.TurretSubsystem;


public class ShootTeleop extends CommandBase {
    TurretSubsystem turret;
    TitanButton increaseSpeedButton;
    TitanButton decreaseSpeedButton;
    double speed = 0;
    public ShootTeleop(TitanButton increaseSpeedButton, TitanButton decreaseSpeedButton, TurretSubsystem turret) {
        this.turret = turret;
        this.increaseSpeedButton = increaseSpeedButton;
        this.decreaseSpeedButton = decreaseSpeedButton;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (increaseSpeedButton.isPressed()){
            if (speed < 0.9){
                speed += 0.1;
            } else {
                speed = 1;
            }
        }
        if (decreaseSpeedButton.isPressed()){
            if (speed > 0.1) {
                speed -= 0.1;
            }
            else {
                speed = 0;
            }
        }
        turret.setShooter(speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.setShooter(0);
    }
}
