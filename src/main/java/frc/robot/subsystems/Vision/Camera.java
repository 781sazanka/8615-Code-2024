package frc.robot.subsystems.Vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cscore.CvSource;

public class Camera extends SubsystemBase {
    public CvSource outputStream;

    public Camera() {
        CameraServer.startAutomaticCapture();
        outputStream = CameraServer.putVideo("Blur", 640, 480);
    }

    public CvSource cameraStream() {
        return outputStream;
    }
}