package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

public class HopperAuto extends CommandBase {

    private Timer timer;
    private final HopperSubsystem hopper;

    public HopperAuto(HopperSubsystem hopper) {
        this.hopper = hopper;
        timer = new Timer();
    }


    @Override
    public void initialize() {
//        this.driveTrain.resetEncoders();
//        this.speedFilter.setValue(0); // reset the filter so that subsequent autonomouses work


//		btnToggleHopperIntake.whileHeld(new HopperIntake(hopper));
//		this.turret.
    }

    @Override
    public void execute() {
        // TODO Filter setpoint at the TitanFX class level so that we don't have to keep filtering stuff
        if (timer.hasElapsed(5)) {
            hopper.intake();
        }

        SmartDashboard.putNumber("timer", timer.get());
    }

    @Override
    public void end(boolean interrupted) {
        this.hopper.stop();
    }

    @Override
    public boolean isFinished() {
        // TODO Enforce max distance here with condition
        return timer.hasElapsed(10);
    }
}
