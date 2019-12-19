package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Robot;
import frc.robot.subsystem.DriveTrain;

public class RobotMath {

    /**
     * get distance per pulse for encoder given certain params
     * @param countsPerRevolution (per wheel cycle)
     * @param wheelRadius the radius of the wheel
     * @return the distance per pulse
     */
    public static double getDistancePerPulse(final double countsPerRevolution, final double wheelRadius) {
        final double encoderAngularDistancePerPulse = 2.0*Math.PI/countsPerRevolution;
        return (wheelRadius * encoderAngularDistancePerPulse)/DriveTrain.GEAR_RATIO_LOW;
    }

    public static double getDistanceFromDegrees(double degrees, double turnRadius) {
        return (2*Math.PI*turnRadius)*(Math.abs(degrees)/360);
    }

    public static double mapDouble(double x, double in_min, double in_max, double out_min, double out_max){
        x = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        
        return x;
    }

    public static double getAngleFromPoint(double x, double y) {
        double robotX = Robot.getInstance().getDrive().getxPos();
        double robotY = Robot.getInstance().getDrive().getyPos();

        double xDiff = x - robotX;
        double yDiff = y - robotY;

        double angle = Math.toDegrees(Math.atan2(xDiff,yDiff));

        // if (angle >= 0 && angle <= 90){
        //     angle = RobotMath.mapDouble(angle, 0, 90, 90, 0);
        // }
        // else if (angle >= 90 && angle <= 180){
        //     angle = RobotMath.mapDouble(angle, 90, 180, 0, -90);
        // }
        // else if (angle >= 180  && angle <= 270){
        //     angle = RobotMath.mapDouble(angle, 180, 270, -90, -180);
        // }
        // else if (angle >= 270 && angle <= 360) {
        //     angle = RobotMath.mapDouble(angle, 270, 360, 180, 90);
        // }

        return angle;
    }

    public static double getDistanceFromPoint(double x, double y){
        double distance;

        double robotX = Robot.getInstance().getDrive().getxPos();
        double robotY = Robot.getInstance().getDrive().getyPos();

        double xDiff = x - robotX;
        double yDiff = y - robotY;

        distance = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));

        return distance;
    }

    public static double MapJoyValues(double value, double low, double high){
        double v = value;

            if (value > 0){
                if ( value > 0.75){
                    value = mapDouble(value, 0.75, 1, high, 1);
                }
                else {
                    value = mapDouble(value, 0, 0.75, low, high);
                }
            }

            else if (value < 0){
                if (value < -0.75){
                    value = mapDouble(value, -1, -0.75, -1, -high);
                }
                else {
                    value = mapDouble(value, -0.75, 0, -high, -low);
                }
            }
        
        
        return value;
    }
}
