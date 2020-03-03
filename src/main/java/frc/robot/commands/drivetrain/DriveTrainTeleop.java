package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motor.Filter;
import frc.robot.subsystems.TankDrive;

import java.util.function.DoubleSupplier;


public class DriveTrainTeleop extends CommandBase {

    public static final double MAX_SPEED = 1.0;
    private boolean filterEnabled;

    private DoubleSupplier leftInput, rightInput;
    private Filter leftFilter, rightFilter;

    private Timer coolingTimer = new Timer();
    private final TankDrive driveTrain;

    public DriveTrainTeleop(TankDrive driveTrain, DoubleSupplier leftInput, DoubleSupplier rightInput) {
        // enable filtering by default
        this(driveTrain, leftInput, rightInput, true);
    }

    public DriveTrainTeleop(TankDrive driveTrain, DoubleSupplier leftInput, DoubleSupplier rightInput, boolean filterEnabled) {
        this.driveTrain = driveTrain;
        this.leftInput = leftInput; // adapted from the `DriveTrain` example // todo explain double supplier
        this.rightInput = rightInput;
        this.filterEnabled = filterEnabled;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        leftFilter = new Filter(0.5); // todo adjust sensitivity
        rightFilter = new Filter(0.5); // todo adjust sensitivity
    }

    @Override
    public void execute() {
        double maxSpeed = MAX_SPEED; // todo actually adjust max speed based on other circumstances
        if (filterEnabled) {
            leftFilter.update(leftInput.getAsDouble());
            rightFilter.update(rightInput.getAsDouble());
            driveTrain.set(leftFilter.getValue(), rightFilter.getValue());
        } else {
            driveTrain.set(leftInput.getAsDouble(), rightInput.getAsDouble());
        }
        SmartDashboard.putBoolean("Compressor value", driveTrain.getPressureSwitchValue());
        double avgTemp = (driveTrain.getLeft().getTemperature() + driveTrain.getRight().getTemperature()) / 2.0;
        if (avgTemp > TankDrive.MAX_MOTOR_TEMP && driveTrain.getPressureSwitchValue()) {
            driveTrain.setCooling(true);
            coolingTimer.start();
        }
        if (coolingTimer.get() > 3) {
            driveTrain.setCooling(false);
            coolingTimer.reset();
            coolingTimer.stop();
        }
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
