package frc.robot.commands.auto;

import frc.robot.Robot;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.TurnToAngle;
import frc.robot.util.RobotMath;
//import ca.team3571.offseason.commands.TurnWithDegrees;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PracticeAuto extends CommandGroup {
    double angle, distance;
    public PracticeAuto() {
       addSequential(new DriveToPoint(0,0));
    }

    @Override
    protected void initialize() {
        System.out.println("Executing practice autonomous");
    }
}
