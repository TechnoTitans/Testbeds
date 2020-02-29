package frc.robot.sensors;

import edu.wpi.first.wpilibj.SerialPort;

import java.nio.ByteBuffer;
import java.util.Arrays;

public class Vision {

    SerialPort visionData;

    public Vision() {
        visionData = new SerialPort(9600, SerialPort.Port.kUSB);
    }

    public double[] getData() {
        double[] data = new double[5];
        int c = 0;

        byte[] visionDataBytes = visionData.read(40); //if no target is detected returns all -1

        for (int i = 0; i < 40; i += 8) {
            data[c] = ByteBuffer.wrap(Arrays.copyOfRange(visionDataBytes, i, i + 7)).getDouble();
            c++;
        }
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
    public double getDistance() {
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

    // TODO MAKE SURE
    public double getHoodAngle() {
        return -9999;
    }


    public double getCurrentOptimalRPM() {
        return getRPMFromDistance(this.getDistance());
    }


}
