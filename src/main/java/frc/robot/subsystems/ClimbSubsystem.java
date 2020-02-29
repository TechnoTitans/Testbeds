package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.motor.TitanSRX;


public class ClimbSubsystem implements Subsystem {

    public static final int TELEOP_DURATION = ((2 * 60) + 15); // (s) teleop period is 2m15s
//    public static final double TIME_TO_ENDGAME = TELEOP_DURATION - 35; // seconds
    public static final double TIME_TO_ENDGAME = 0; // seconds
    public static final double WINCH_SPEED = 0.3;

    private TitanSRX motor;
    private Solenoid climbSolenoid;
    private Timer endgameTimer;
    private boolean hasReleasedMech = false;

    public ClimbSubsystem(TitanSRX pullUpMotor, Solenoid climbSolenoid) {
        this.motor = pullUpMotor;
        // explicitly make pullUpMotor brake mode
        this.motor.setNeutralMode(NeutralMode.Brake);
        this.climbSolenoid = climbSolenoid;
        endgameTimer = new Timer();
        init();
    }

    /**
     * THIS MUST BE CALLED!!!!
     */
    public void init() {
        endgameTimer.start();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Climb] Avengers Endgame Timer", endgameTimer.get());
        SmartDashboard.putBoolean("[Climb] has mech released", hasReleasedMech);
    }

    public void resetEndgameTimer() {
        endgameTimer.reset();
    }

    public boolean areWeInTheEndGame() {
        return endgameTimer.get() >= TIME_TO_ENDGAME;
    }


    public void stopMotor() {
        motor.set(0);
    }

    private void setMotorSafe(double speed) {
        if (this.hasReleasedMech && areWeInTheEndGame()) {
            motor.set(speed);
        } else {
            System.err.println("Tried to move motor while not in endgame or mech not released.");
        }
    }

    public void setMotorPositive() {
        this.setMotorSafe(+WINCH_SPEED);
    }

    public void setMotorNegative() {
        this.setMotorSafe(-WINCH_SPEED);
    }

    public void releaseMech() {
        if (areWeInTheEndGame()) {
            climbSolenoid.set(true);
            this.hasReleasedMech = true;
        }
    }


}

