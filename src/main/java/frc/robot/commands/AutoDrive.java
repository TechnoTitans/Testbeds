package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;
import edu.wpi.first.wpilibj.Timer;

public class AutoDrive extends CommandBase {

    private final TankDrive driveTrain;
    private final Timer timer;


    public AutoDrive(TankDrive driveTrain, Timer timer) {
        this.driveTrain = driveTrain;
        this.timer = timer;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        //Turn on motor
        this.driveTrain.getGyro().reset();
        this.driveTrain.getGyro().calibrate();

        //Start Timer
        timer.start();
    }

    @Override
    public void execute() {
        //Drive while timer is less than 5 sec
        do{
            this.driveTrain.set(0.5);
        }
        while(!timer.hasElapsed(5));

    }

    @Override
    public void end(boolean interrupted) {
        this.driveTrain.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
