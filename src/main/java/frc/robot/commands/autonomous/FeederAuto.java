//package frc.robot.commands.auto;
//
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpiutil.math.MathUtil;
//import frc.robot.motor.Filter;
//import frc.robot.subsystems.FeederSubsystem;
//import frc.robot.subsystems.HopperSubsystem;
//import frc.robot.subsystems.TankDrive;
//import frc.robot.subsystems.TurretSubsystem;
//
//
//public class FeederAuto extends CommandBase {
//    private final FeederSubsystem feeder;
//    private int timer;
//
//    public FeederAuto(FeederSubsystem feeder) {
//        this.feeder = feeder;
//    }
//
//
//    @Override
//    public void initialize() {
//        this.driveTrain.resetEncoders();
//        this.speedFilter.setValue(0); // reset the filter so that subsequent autonomouses work
//
//
////		btnToggleHopperIntake.whileHeld(new HopperIntake(hopper));
////		this.turret.
//    }
//
//    @Override
//    public void execute() {
//        // TODO Filter setpoint at the TitanFX class level so that we don't have to keep filtering stuff
//        feeder.setBelt(1);
//
//        SmartDashboard.putNumber("timer", timer);
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        this.feeder.setBelt();
//    }
//
//    @Override
//    public boolean isFinished() {
//        // TODO Enforce max distance here with condition
//        if (timer > 500) {
//            return driveTrain.getRightEncoder().getDistance() <= this.desiredDistance;
//
//        } else {
//            return driveTrain.getRightEncoder().getDistance() >= this.desiredDistance;
//        }
//    }
//}
