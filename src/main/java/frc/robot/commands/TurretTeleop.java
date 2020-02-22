package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motor.Filter;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TurretTeleop extends CommandBase {
    private double speed;
    private TurretSubsystem turret;
    private final double min = 0;
    private final double max = 0;
    private int reversed = 1;
    private boolean filterEnabled;
    private Filter hoodFilter;
    private Filter zFilter;
    DoubleSupplier zMotorInput, hoodMotorInput;
    public TurretTeleop(DoubleSupplier zMotorInput, DoubleSupplier hoodMotorInput, TurretSubsystem turret) {
        // If any subsystems are needed, you will need to pass them into the requires() method
        addRequirements(turret);
        this.zMotorInput = zMotorInput;
        this.hoodMotorInput = hoodMotorInput;
        this.turret = turret;
    }
    public TurretTeleop(DoubleSupplier zMotorInput, DoubleSupplier hoodMotorInput, TurretSubsystem turret, boolean filterEnabled) {
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
            turret.setHood(hoodFilter.getValue());
        }
        else {
            turret.setZMotor(zMotorInput.getAsDouble());
            turret.setHood(hoodMotorInput.getAsDouble());
        }
        turret.setShooter(turret.getPercentOutputSetpoint());  // this is set by the buttonbindings
        SmartDashboard.putNumber("Turret Shooter Speed", turret.getPercentOutputSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        turret.setZMotor(0);
        turret.setHood(0);
        turret.setShooter(0);
    }

    @Override
    public boolean isFinished() {
//        return turret.getZMotorEncoder().getDistance() >= max || turret.getZMotorEncoder().getDistance() <= min;
        return false;
    }
}
