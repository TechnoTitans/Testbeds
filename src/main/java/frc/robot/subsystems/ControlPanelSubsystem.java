package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.motor.TitanSRX;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;

import javax.xml.crypto.Data;

public class ControlPanelSubsystem extends SubsystemBase {

    private TitanSRX spinningMotor;
    private I2C.Port i2cPort = I2C.Port.kOnboard;
    private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    private final Color BlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color GreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color RedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color YellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


    public ControlPanelSubsystem(TitanSRX spinningMotor, ColorSensorV3 colorSensor) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        this.spinningMotor = spinningMotor;
        this.colorSensor = colorSensor;
        spinningMotor.getEncoder();
    }

    public void setSpeed(double speed) {
        if (speed > 1) {
            spinningMotor.set(ControlMode.PercentOutput, 1);
        } else if (speed < -1) {
            spinningMotor.set(ControlMode.PercentOutput, -1);
        } else {
            spinningMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    public void moveRotations(double rotations) {
        rotations *= 4096;
        spinningMotor.set(rotations);
    }

    public String findColor() {
        Color detectedColor = colorSensor.getColor();
        String colorString;
        if (detectedColor == BlueTarget) {
            colorString = "Blue";
        } else if (detectedColor == RedTarget) {
            colorString = "Red";
        } else if (detectedColor == GreenTarget) {
            colorString = "Green";
        } else if (detectedColor == YellowTarget) {
            colorString = "Yellow";
        } else {
            colorString = "Unknown";
        }
        SmartDashboard.putString("Detected Color", colorString);
        return colorString;
    }

    public ColorSensorV3.RawColor getColor(){
        ColorSensorV3.RawColor rawColor = colorSensor.getRawColor();

        return colorSensor.getRawColor();
    }

    public String getFMSColor(){
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B' :
                    gameData = "Blue";
                    break;
                case 'G' :
                    gameData = "Green";
                    break;
                case 'R' :
                    gameData = "Red";
                    break;
                case 'Y' :
                    gameData = "Yellow";
                    break;
                default :
                    //This is corrupt data
                    break;
            }
        } else {
            gameData = "No data received";
        }
        return gameData;
    }
}