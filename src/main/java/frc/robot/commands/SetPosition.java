package frc.robot.commands;

import frc.robot.Robot;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SetPosition extends Command {
    double xPos;
    double yPos;

    public SetPosition(double x, double y) {
        xPos = x;
        yPos = y;
        requires(Robot.getInstance().getDrive());
    }

    @Override
    protected void initialize() {
        Robot.getInstance().getDrive().setxPos(xPos);
        Robot.getInstance().getDrive().setyPos(xPos);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }
}
