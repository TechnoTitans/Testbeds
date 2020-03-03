package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;


public class DriveTrainAutonomous extends CommandBase {

    private double distance;
    TankDrive drive;

    public DriveTrainAutonomous(TankDrive drive, double distance) {
        this.distance = distance; // computations here
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.getLeftEncoder().reset();
        drive.getPID().setSetpoint(distance);
    }

    @Override
    public void execute() {
        drive.set(drive.getPID().calculate(drive.getLeftEncoder().getDistance()));
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return drive.getPID().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.set(0);
    }
}
