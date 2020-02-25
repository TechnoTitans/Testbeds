package frc.robot.sensors;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class DriverCamera {
    private UsbCamera cam1;
    private CvSink cvSink;
    private CvSource outputStream;
    private Mat source;
    private Mat output;


//    private UsbCamera cam1, cam2;

    //    private VideoSink sink;
    public DriverCamera() {
        init();
//        cam2 = CameraServer.getInstance().startAutomaticCapture(1);
//        sink = CameraServer.getInstance().getServer();
//        cam1 = CameraServer.getInstance().getVideo();
//        outputStream = CameraServer.getInstance().putVideo("LineDetector Stream", 640, 480);

    }

    private void init() {
        cam1 = CameraServer.getInstance().startAutomaticCapture(0);
        cam1.setResolution(320 / 2, 240 / 2);

        cvSink = CameraServer.getInstance().getVideo();
        outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

        source = new Mat();
        output = new Mat();
    }


    public void updateImage() {
        cvSink.grabFrame(source);
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
    }
}
