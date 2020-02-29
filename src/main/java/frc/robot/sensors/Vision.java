package frc.robot.sensors;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.ByteBuffer;
import java.util.Arrays;

public class Vision {

    SerialPort visionData;

    /**
     * The number of values (doubles) we'll be recieving from vision
     */
    private static final int NUM_VALUES = 5;
    private static final int BYTES_FROM_DEVICE = NUM_VALUES * 8;

    public Vision() {
        visionData = new SerialPort(115200, SerialPort.Port.kUSB);
    }

    // TODO make this work
    public double[] getData() {
        double[] data = new double[5];
        int c = 0;

        byte[] visionDataBytes = visionData.read(BYTES_FROM_DEVICE); //if no target is detected returns all -1

        for (int i = 0; i < 40; i += 8) {
            data[c] = ByteBuffer.wrap(Arrays.copyOfRange(visionDataBytes, i, i + 8)).getDouble();
            c++;
        }
        SmartDashboard.putString("Byte Array of Vision Data", Arrays.toString(visionDataBytes));
        SmartDashboard.putNumberArray("Double Array of Vision Data", (data));

        return data;
    }

    //todo set default values
    public double getAngleX() {
        return getData()[3];
    }

    public double getAngleY() {
        return getData()[4];
    }

    /**
     * Returns distance from target in inches
     *
     * @return distance in inches
     */
    public double getVisionDistance() {
        return getData()[2];
    }

    public double getCenterX() {
        return getData()[0];
    }

    public double getCenterY() {
        return getData()[1];
    }

    /**
     * @param distance - distance from targets in inches
     * @return the rpm that the flywheel must go to shoot well
     */
    public double getRPMFromDistance(double distance) {
        // from curve fit
        double A = 1155,
                B = 5.433,
                C = 0.1015,
                D = -0.0003018;

        return A + B * distance + C * Math.pow(distance, 2) + D * Math.pow(distance, 3);
    }

    /**
     *
     * @param distance distance away from target in inches
     * @return optimal encoder position for hood
     */
    public double getHoodAngle(double distance) {
        double A = 3482,
                B = -63.07,
                C = 0.3093,
                D = -0.00005104;

        return A + B * distance + C * Math.pow(distance, 2) + D * Math.pow(distance, 3);
    }


    public double getCurrentOptimalRPM() {
        return getRPMFromDistance(this.getVisionDistance());
    }

    /**
     *
     * @return - optimal hood position in encoder ticks
     */
    public double getCurrentOptimalHoodAngle() {
        return getHoodAngle(this.getVisionDistance());
    }


}
