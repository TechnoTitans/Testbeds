package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.TurretSubsystem;



public class TurretAutonomous extends CommandBase {
    private double xAngle;
    private double yAngle;
    private double distance;
    private double shooterSpeed;
    private Vision vision;
    private TurretSubsystem turret;
    public TurretAutonomous(Vision vision, TurretSubsystem turret) {
        this.turret = turret;
        this.vision = vision;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        xAngle = vision.getAngleX(); //todo calculations here to convert from angles to encoder
        yAngle = vision.getAngleY(); //decide if angle should be the lower angle or larger, then convert from angle to encoder
        distance = vision.getDistance(); // units in feet
        //Decide shooter speed here
        if (distance > 10){
            shooterSpeed = 0.8;
        }

//        turret.getZMotorPID().setSetpoint(xAngle);
//        turret.getHoodPID().setSetpoint(yAngle);
//        turret.setHood(turret.getHoodPID().calculate(turret.getHoodEncoder().getDistance()));
//        turret.setZMotor(turret.getZMotorPID().calculate(turret.getZMotorEncoder().getDistance()));
//        turret.setShooter(shooterSpeed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
//        return turret.getZMotorPID().atSetpoint() && turret.getHoodPID().atSetpoint() && turret.getShooter().getSpeed() >= shooterSpeed;
        return false;
        //this command is issue by a button and will be canceled by another button that starts teleop commands
    }

    @Override
    public void end(boolean interrupted) {
        turret.setZMotor(0);
        turret.setHood(0);
        turret.setShooter(0);
    }
}
