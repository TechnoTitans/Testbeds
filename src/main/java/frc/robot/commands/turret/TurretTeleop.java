package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motor.Filter;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TurretTeleop extends CommandBase {


    private boolean filterEnabled;

    private Filter hoodFilter;
    private Filter zFilter;
    DoubleSupplier zMotorInput, hoodMotorInput;
    private TurretSubsystem turret;

    public TurretTeleop(TurretSubsystem turret, DoubleSupplier zMotorInput, DoubleSupplier hoodMotorInput) {
        // If any subsystems are needed, you will need to pass them into the requires() method
        addRequirements(turret);
        this.zMotorInput = zMotorInput;
        this.hoodMotorInput = hoodMotorInput;
        this.turret = turret;
    }
    public TurretTeleop(TurretSubsystem turret, DoubleSupplier zMotorInput, DoubleSupplier hoodMotorInput, boolean filterEnabled) {
        // If any subsystems are needed, you will need to pass them into the requires() method
        addRequirements(turret);
        this.zMotorInput = zMotorInput;
        this.hoodMotorInput = hoodMotorInput;
        this.turret = turret;
        this.filterEnabled = filterEnabled;
    }


    @Override
    public void initialize() {
        hoodFilter = new Filter(0.7);
        zFilter = new Filter(0.7);
    }

    @Override
    public void execute() {
        if (filterEnabled){
            hoodFilter.update(hoodMotorInput.getAsDouble());
            zFilter.update(zMotorInput.getAsDouble());
            turret.setZMotor(zFilter.getValue());
//            turret.setHood(hoodFilter.getValue()); // this is now controlled by presets
        }
        else {
            turret.setZMotor(zMotorInput.getAsDouble());
//            turret.setHood(hoodMotorInput.getAsDouble());
        }
        turret.setShooterVelocityRPM(turret.getRPMSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        turret.setZMotor(0);
        turret.setHood(0);
        turret.setShooter(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
