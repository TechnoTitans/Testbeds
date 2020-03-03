package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.TurretSubsystem;



public class TurretAutonomous extends CommandBase {

    private double shooterSpeed;
    private Vision vision;
    private TurretSubsystem turret;

    public TurretAutonomous(TurretSubsystem turret, Vision vision) {
        this.turret = turret;
        this.vision = vision;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double xAngle = vision.getAngleX();
        double yAngle = vision.getAngleY();
        double distance = vision.getVisionDistance(); // units in feet
        double desiredHoodAngle = 0; //todo find out turret angles based on vision
        double desiredTurretAngle = 0;
        //Decide shooter speed here
        turret.setHoodAngle(desiredHoodAngle);
        turret.setTurretAngle(desiredTurretAngle);

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
