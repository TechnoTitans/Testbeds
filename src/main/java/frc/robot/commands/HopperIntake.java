package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

public class HopperIntake extends CommandBase {

    final HopperSubsystem hopper;

    public HopperIntake(HopperSubsystem hopper) {
        this.hopper = hopper;
    }

    @Override
    public void execute() {
        hopper.intake();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) { hopper.stop(); }
}
