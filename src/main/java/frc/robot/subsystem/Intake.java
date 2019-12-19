package frc.robot.subsystem;

import frc.robot.Robot;
import frc.robot.util.Loggable;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem implements Loggable, Refreshable {

    private static int FIRST_MOTOR_PORT = 3;
    private static int SECOND_MOTOR_PORT = 4;

    private Spark firstMotor;
    private Spark secondMotor;

    public Intake() {
        firstMotor = new Spark(FIRST_MOTOR_PORT);
        secondMotor = new Spark(SECOND_MOTOR_PORT);

        firstMotor.setInverted(false);
        secondMotor.setInverted(true);
    }

    @Override
    public void refresh() {
        if(Math.abs(Robot.getInstance().getSubsystemController().Triggers.Left) > 0) {
            firstMotor.setSpeed(Robot.getInstance().getSubsystemController().Triggers.Left);
            secondMotor.setSpeed(Robot.getInstance().getSubsystemController().Triggers.Left);
        }
        
        else {
            firstMotor.setSpeed(-Robot.getInstance().getSubsystemController().Triggers.Right);
            secondMotor.setSpeed(-Robot.getInstance().getSubsystemController().Triggers.Right);
        }
    }

    @Override
    public void log() {

    }

    @Override
    protected void initDefaultCommand() {

    }
}
