/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;/**
 * Add your docs here.
 */
public class LiftManualCommand extends Command {
    private static final double LIFT_SPEED = 0.6;
    private double speed;

    public LiftManualCommand(boolean up) {
        requires(Robot.getInstance().getElevator());
        if(up) {
            speed = LIFT_SPEED;
        }
        else {
            speed = -LIFT_SPEED;
        }
    }

    @Override
    protected void initialize() {
        Robot.getInstance().getElevator().getElevatorMotor().setSpeed(speed);
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    protected void end() {
        Robot.getInstance().getElevator().getElevatorMotor().setSpeed(0);
    }
}