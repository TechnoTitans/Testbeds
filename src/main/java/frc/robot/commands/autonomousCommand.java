package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;
import frc.robot.motor.TitanSRX;

public class ExampleCommand extends CommandBase {

    private final TankDrive driveTrain;
    TitanSRX mainMotor;
    private final int MAX_DISTANCE = 60;

    public autonomousCommand() {
        //this.driveTrain = driveTrain;
        //addRequirements(driveTrain);
    }

    @Override
    public void initialize() {

        //this.driveTrain.getGyro().reset();
        mainMotor.reset();
    }

    @Override
    public void execute() {

        //this.driveTrain.set(0.5);
        mainMotor.set(5);

        mainMotor.set(-5);
    }

    @Override
    public void end(boolean interrupted) {

        //this.driveTrain.brake();
    }

    @Override
    public boolean isFinished() {
//        return this.driveTrain.getLeftEncoder().getDistance() > MAX_DISTANCE;
        return true;
    }
}
