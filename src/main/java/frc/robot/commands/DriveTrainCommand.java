package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;


public class DriveTrainCommand extends CommandBase {
    private final DriveTrain driveTrain;

    private DoubleSupplier leftInput, rightInput;
//    private Filter leftFilter, rightFilter; // todo implement filtering

    public DriveTrainCommand(DoubleSupplier leftInput, DoubleSupplier rightInput, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.leftInput = leftInput; // adapted from the `DriveTrain` example
        this.rightInput = rightInput;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.set(leftInput.getAsDouble(), rightInput.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.driveTrain.stop();
    }
}
