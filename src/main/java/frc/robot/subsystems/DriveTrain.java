package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class DriveTrain {

    private TalonFX left;
    private TalonFX right;

    public DriveTrain(TalonFX leftFTalonFX, TalonFX rightFTalonFX) {
        this(leftFTalonFX, rightFTalonFX);
    }

    public DriveTrain(TalonFX leftFTalonFX, TalonFX rightFTalonFX) {
        this.left = leftFTalonFX;
        this.right = rightFTalonFX;
    }

    public void set(double speed) {
        left.set(speed);
        right.set(speed);
    }

}

