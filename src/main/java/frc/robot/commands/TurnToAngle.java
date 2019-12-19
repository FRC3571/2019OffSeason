package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystem.DriveTrain;
import frc.robot.util.RobotMath;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;

public class TurnToAngle extends Command implements PIDOutput {

    private PIDController turnController;
    private double rotateToAngleRate;
    private AHRS ahrs;
    private double degrees;

    // PID Constants
    private static final double kP = 0.015;
    private static final double kI = 0.000;
    private static final double kD = 0.00;
    private static final double kF = 0.00;

    private static final double kToleranceDegrees = 2.0f;

    public TurnToAngle(double d, AHRS a) {
        ahrs = a;
        degrees = d;

        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180.0f, 180.0f);
        turnController.setOutputRange(-1, 1);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);

        turnController.setSetpoint(degrees);

        requires(Robot.getInstance().getDrive());
    }

    public TurnToAngle(double x, double y, AHRS a) {
        ahrs = a;
        degrees = RobotMath.getAngleFromPoint(x, y);

        if (Math.abs(degrees - Robot.getInstance().getNAVX().getYaw()) > 90){
            if (degrees >= 0 && degrees <= 180){
                degrees -= 180;
            }
            else {
                degrees += 180;
            }
        }

        System.out.println("ANGLE IS" + degrees);

        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180.0f, 180.0f);
        turnController.setOutputRange(-1, 1);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);


        turnController.setSetpoint(degrees);

        requires(Robot.getInstance().getDrive());
    }

    @Override
    protected void initialize() {
        Robot.getInstance().getDrive().reset();

        Robot.getInstance().getDrive().setChosenGear(DriveTrain.Gear.FOURTH);
        
        turnController.enable();
    }

    @Override
    protected void execute() {
        //if (rotateToAngleRate < 0.15) rotateToAngleRate -= 0.7;
        //else rotateToAngleRate += 0.55;
        if (rotateToAngleRate < 0) {
             rotateToAngleRate = RobotMath.mapDouble(rotateToAngleRate, -1.0, 0.0, -0.5, -0.2);
         }
        else {
             rotateToAngleRate = RobotMath.mapDouble(rotateToAngleRate, 0, 1, 0.2, 0.5);
         }

        Robot.getInstance().getDrive().arcadeDrive(0, -rotateToAngleRate);
    }

    @Override
    protected boolean isFinished() {
        return turnController.onTarget();
    }

    @Override
    protected void end() {
        Robot.getInstance().getDrive().arcadeDrive(0, 0);
        Robot.getInstance().getDrive().setChosenGear(DriveTrain.Gear.SECOND);
        turnController.disable();
    }

    @Override
    public void pidWrite(double output) {
        rotateToAngleRate = output;
    }
}