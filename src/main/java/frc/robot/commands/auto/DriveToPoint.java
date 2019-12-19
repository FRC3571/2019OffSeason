package frc.robot.commands.auto;

import frc.robot.Robot;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.TurnToAngle;
import frc.robot.util.RobotMath;
//import ca.team3571.offseason.commands.TurnWithDegrees;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveToPoint extends CommandGroup {
    double xTarget, yTarget, distance;
    public DriveToPoint(double x, double y) {
        xTarget = x;
        yTarget = y;

        addSequential(new TurnToAngle(xTarget, yTarget, Robot.getInstance().getNAVX()));
        addSequential(new DriveStraightDistance(xTarget, yTarget));
    }
}
