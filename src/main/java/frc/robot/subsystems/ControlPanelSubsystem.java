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
    private final double COLOR_ERROR_THRESHOLD = 0.2;

    private TitanSRX spinningMotor;
    private I2C.Port i2cPort = I2C.Port.kOnboard;
    private ColorSensorV3 colorSensor;

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

    public void moveToColor() {
        String color = getFMSColor();
        if (!findClosestColor().equals(color)){
            setSpeed(.5);
        } else {
            setSpeed(0);
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

    public String findClosestColor() {
        Color detectedColor = colorSensor.getColor();
        double error = 10;

        String closestColor = "Unknown";

        if (calculateError(detectedColor, BlueTarget) < error && calculateError(detectedColor, BlueTarget) < COLOR_ERROR_THRESHOLD) {
            error = calculateError(detectedColor, BlueTarget);
            closestColor = "Blue";
        } else if (calculateError(detectedColor, GreenTarget) < error && calculateError(detectedColor, GreenTarget) < COLOR_ERROR_THRESHOLD) {
            error = calculateError(detectedColor, GreenTarget);
            closestColor = "Green";
        } else if (calculateError(detectedColor, YellowTarget) < error && calculateError(detectedColor, YellowTarget) < COLOR_ERROR_THRESHOLD) {
            error = calculateError(detectedColor, YellowTarget);
            closestColor = "Yellow";
        } else if (calculateError(detectedColor, RedTarget) < error && calculateError(detectedColor, RedTarget) < COLOR_ERROR_THRESHOLD) {
            error = calculateError(detectedColor, RedTarget);
            closestColor = "Red";
        }

        return closestColor;
    }

    private double calculateError(Color detected, Color target) {
        double redError = Math.abs(detected.red - target.red);
        double greenError = Math.abs(detected.green - target.green);
        double blueError = Math.abs(detected.blue - target.blue);
        return redError + greenError + blueError;
    }

    public Color getColor(){
//        ColorSensorV3.RawColor rawColor = colorSensor.getRawColor();
//        colorSensor.getColor().

        return colorSensor.getColor();
    }

    public String getFMSColor(){
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0) {
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