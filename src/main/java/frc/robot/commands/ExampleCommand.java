package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

public class ExampleCommand extends CommandBase {

    private final TankDrive driveTrain;
    private final int MAX_DISTANCE = 60;

    public ExampleCommand(TankDrive driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        this.driveTrain.getGyro().reset();
    }

    @Override
    public void execute() {
        this.driveTrain.set(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        this.driveTrain.brake();
    }

    @Override
    public boolean isFinished() {
//        return this.driveTrain.getLeftEncoder().getDistance() > MAX_DISTANCE;
        return true;
    }
}
