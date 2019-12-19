package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystem.DriveTrain;
import frc.robot.util.XboxController;
import edu.wpi.first.wpilibj.command.Command;

public class DriveJoystick extends Command {

    private DriveTrain driveTrain;
    private XboxController controller;

    public DriveJoystick(XboxController controller) {
        this.driveTrain = Robot.getInstance().getDrive();
        this.controller = controller;
        requires(driveTrain);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        driveTrain.drive(controller);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false; // Runs until interrupted
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        driveTrain.arcadeDrive(0, 0);
    }
}
