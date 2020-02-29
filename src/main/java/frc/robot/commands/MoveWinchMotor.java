package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

import java.util.Objects;

public class MoveWinchMotor extends CommandBase {


    public static enum Direction {
        POSITIVE,
        NEGATIVE
    }

    private final ClimbSubsystem climb;
    private final Direction desiredMotorDirection;



    public MoveWinchMotor(ClimbSubsystem climbSubsystem, Direction motorDirection) {
        super();
        Objects.requireNonNull(climbSubsystem);
        Objects.requireNonNull(motorDirection);

        this.climb = climbSubsystem;
        this.desiredMotorDirection = motorDirection;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        if (this.desiredMotorDirection.equals(Direction.POSITIVE)) {
            this.climb.setMotorPositive();
        } else if (this.desiredMotorDirection.equals(Direction.NEGATIVE)) {
            this.climb.setMotorNegative();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.climb.stopMotor();
    }
}
