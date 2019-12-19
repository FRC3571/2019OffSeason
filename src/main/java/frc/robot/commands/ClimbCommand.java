package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class ClimbCommand extends Command {

    private static final int SOLENOID_PORT = 0;
    private static final int FIRST_ID = 0;
    private static final int SECOND_ID = 7;


    public ClimbCommand() {
        requires(Robot.getInstance().getPneumatics());
        //climb
        Robot.getInstance().getPneumatics().createSolenoid(SOLENOID_PORT, FIRST_ID, SECOND_ID);
    }

    @Override
    public void initialize() {
        
        if(Robot.getInstance().getPneumatics().getOpenState()) {
            Robot.getInstance().getPneumatics().solenoidReverse(SOLENOID_PORT);
            Robot.getInstance().getPneumatics().setOpenState(false);
        }
        else {
            Robot.getInstance().getPneumatics().solenoidForward(SOLENOID_PORT);
            Robot.getInstance().getPneumatics().setOpenState(true);
        }
        
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
