package frc.robot.component;

import edu.wpi.cscore.UsbCamera;

public class RobotCamera {

    private UsbCamera camera;

    public RobotCamera(UsbCamera camera) {
        this.camera = camera;
    }

    public void setAsPriority() {
        camera.setFPS(20);
        camera.setResolution(640, 480);
    }

    public void removePriority() {
        camera.setFPS(10);
        camera.setResolution(160, 120);
    }
}
