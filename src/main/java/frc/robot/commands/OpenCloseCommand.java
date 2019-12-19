/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Add your docs here.
 */
public class OpenCloseCommand extends Command {
    private static final int SOLENOID_PORT = 1; //intake solenoid
    private static final int FIRST_ID = 1;
    private static final int SECOND_ID = 6;

    public OpenCloseCommand() {
        requires(Robot.getInstance().getPneumatics());
        //intake
        Robot.getInstance().getPneumatics().createSolenoid(SOLENOID_PORT, FIRST_ID, SECOND_ID);
    }

    @Override
    public void initialize() {
        
        if(Robot.getInstance().getPneumatics().getIntakeOpenState()) {
            Robot.getInstance().getPneumatics().solenoidReverse(SOLENOID_PORT);
            Robot.getInstance().getPneumatics().setIntakeOpenState(false);
        }
        else {
            Robot.getInstance().getPneumatics().solenoidForward(SOLENOID_PORT);
            Robot.getInstance().getPneumatics().setIntakeOpenState(true);
        }
        
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}