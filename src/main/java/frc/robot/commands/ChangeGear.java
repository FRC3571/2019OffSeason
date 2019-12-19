package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystem.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

public class ChangeGear extends Command {
    private double gear;

    public ChangeGear(double d) {
        gear = d;
    }

    @Override
    protected void initialize() {
        if (gear == 1){
            Robot.getInstance().getDrive().setChosenGear(DriveTrain.Gear.FIRST);
        }
        else if (gear == 2){
            Robot.getInstance().getDrive().setChosenGear(DriveTrain.Gear.SECOND);
        }
        else if (gear == 3){
            Robot.getInstance().getDrive().setChosenGear(DriveTrain.Gear.THIRD);
        }
        else {
            Robot.getInstance().getDrive().setChosenGear(DriveTrain.Gear.FOURTH);
        }
    }
    
    @Override
    protected boolean isFinished() {
        return true;
    }
}
