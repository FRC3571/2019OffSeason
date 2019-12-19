package frc.robot.component;

import java.util.ArrayList;


public class CameraController {

    //private RobotCamera frontCamera;
    //private RobotCamera rearCamera;

    private ArrayList<RobotCamera> cameraList = new ArrayList<>();  //initial size is 10

    public CameraController() {
    }

    public void addCamera(RobotCamera camera){
        cameraList.add(camera);
    }

//    public CameraController(RobotCamera frontCamera, RobotCamera rearCamera) {
//        this.frontCamera = frontCamera;
//        this.rearCamera = rearCamera;
//    }

    public void begin() {
        //default with front and rear prioritized
        //frontCamera.setAsPriority();
        //rearCamera.setAsPriority();

        //set all cameras to priority
        for(int i = 0; i < cameraList.size();i++){
            cameraList.get(i).setAsPriority();
        }
    }

    public void setPriority(int cameraNumber) {
        for(int i = 0; i < cameraList.size(); i++){
            if(i == cameraNumber){
                cameraList.get(i).setAsPriority();
            } else {
                cameraList.get(i).removePriority();
            }
        }

        //frontCamera.setAsPriority();
        //rearCamera.removePriority();
    }

//    public void setRearPriority() {
//        rearCamera.setAsPriority();
//        frontCamera.removePriority();
//    }
}
