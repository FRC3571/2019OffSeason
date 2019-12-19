package frc.robot.subsystem;

import frc.robot.Robot;
import frc.robot.util.Loggable;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Tilt extends Subsystem implements Loggable, Refreshable {

    private static int ENCODER_CHANNEL_A;
    private static int ENCODER_CHANNEL_B;

    private static int MOTOR_PORT = 5;
    private static final double SPEED = 0.4;

    private boolean isReverse = false;

    private Encoder encoder;
    private Spark tiltMotor;

    public Tilt() {
        //encoder = new Encoder(ENCODER_CHANNEL_A, ENCODER_CHANNEL_B, isReverse, CounterBase.EncodingType.k1X);
        tiltMotor = new Spark(MOTOR_PORT);
        tiltMotor.setInverted(false);
    }

    @Override
    public void refresh() {
        //TODO do real-time calculation && movements here
        double stickPos = Robot.getInstance().getSubsystemController().RightStick.Y;
        tiltMotor.setSpeed(stickPos > 0 ? SPEED : stickPos < 0 ? -SPEED : 0);
    }

    @Override
    public void log() {

    }

    @Override
    protected void initDefaultCommand() {

    }

    public Spark getTiltMotor() {
        return tiltMotor;
    }
}
