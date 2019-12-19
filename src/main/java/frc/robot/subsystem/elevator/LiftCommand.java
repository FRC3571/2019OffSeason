package frc.robot.subsystem.elevator;


import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class LiftCommand extends Command {

   private boolean up;
   private boolean finished;
   private double targetDistance;
   private double speed = 0.4;


    public LiftCommand(boolean up) {
        requires(Robot.getInstance().getElevator());
        this.up = up;
    }

    @Override
    public void initialize() {
        Robot.getInstance().getElevator().getDistanceEncoder().reset();
        ElevatorStage currentStage = Robot.getInstance().getElevator().getStage();
        if(currentStage == ElevatorStage.BOTTOM) {
            if(up) {
                targetDistance = 10000;
                Robot.getInstance().getElevator().setStage(ElevatorStage.MIDDLE);
            }
            else {
                System.out.println("You Can't Go Lower");
                targetDistance = 0;
            }
        }
        else if(currentStage == ElevatorStage.MIDDLE) {
            if(up) {
                targetDistance = 15000;
                Robot.getInstance().getElevator().setStage(ElevatorStage.TOP);
            }
            else {
                targetDistance = 10000;
                Robot.getInstance().getElevator().setStage(ElevatorStage.BOTTOM);
            }
        }
        else {
            if(up) {
                System.out.println("You Can't Go Higher");
                targetDistance = 0;
            }
            else {
                targetDistance = 15000;
                Robot.getInstance().getElevator().setStage(ElevatorStage.MIDDLE);
            }
        }
    }

    @Override
    protected void execute() {
        double currentDistance = Math.abs(Robot.getInstance().getElevator().getDistance());
        if(currentDistance >= targetDistance) {
            finished = true;
        }
        else {
            Robot.getInstance().getElevator().getElevatorMotor().setSpeed(speed);
        }
    }

    @Override
    protected boolean isFinished() {
        if(finished) {
            targetDistance = 0;
            Robot.getInstance().getElevator().getElevatorMotor().setSpeed(0);
            finished = false;
            return true;
        }
        return false;
    }
}
