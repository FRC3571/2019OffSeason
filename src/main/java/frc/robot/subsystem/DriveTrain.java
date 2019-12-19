package frc.robot.subsystem;

import frc.robot.Robot;
import frc.robot.commands.ChangeGear;
import frc.robot.commands.DriveJoystick;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.SetPosition;
import frc.robot.commands.TurnToAngle;
import frc.robot.util.Loggable;
import frc.robot.util.RobotMath;
import frc.robot.util.XboxController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
//import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;

public class DriveTrain extends PIDSubsystem implements Loggable, Refreshable {

    // motor ports
    private static int LEFT_MOTOR_GROUP, RIGHT_MOTOR_GROUP;
    // encoder ports/channels
    // private static int FRONT_LEFT_ENCODER_CHANNEL_A,
    // FRONT_LEFT_ENCODER_CHANNEL_B,
    // FRONT_RIGHT_ENCODER_CHANNEL_A,
    // FRONT_RIGHT_ENCODER_CHANNEL_B;

    // private static boolean FORWARD_DIRECTION, REVERSE_DIRECTION;
    // private static CounterBase.EncodingType ENCODER_TYPE;

    // encoder mapping
    // private static int COUNTS_PER_REVOLUTION;
    // private static double WHEEL_RADIUS;

    // controller port
    private static int CONTROLLER_PORT;

    // gear ratios
    public static final double GEAR_RATIO_LOW;
    public static final double GEAR_RATIO_HIGH;

    // SparkMax
    private static final int LEFTID;
    private static final int RIGHTID;

    private double distance;

    //Choose Drive Mode
    private enum DriveMode {
        AONEJOY,
        ATWOJOY,
        TANK,
    }

    public enum Gear {
        FIRST,
        SECOND,
        THIRD,
        FOURTH,
    }

    private DriveMode ChosenDrive;
    private Gear ChosenGear;
    private SendableChooser<DriveMode> DriveModeChooser;

    static {
        // initialization
        // LEFT_MOTOR_GROUP = 0;
        // RIGHT_MOTOR_GROUP = 1;

        // FRONT_LEFT_ENCODER_CHANNEL_A = 2;
        // FRONT_LEFT_ENCODER_CHANNEL_B = 3;
        // FRONT_RIGHT_ENCODER_CHANNEL_A = 4;
        // FRONT_RIGHT_ENCODER_CHANNEL_B = 5;

        // FORWARD_DIRECTION = false;
        // REVERSE_DIRECTION = true;
        // ENCODER_TYPE = CounterBase.EncodingType.k1X;
        // COUNTS_PER_REVOLUTION = 2048;
        // WHEEL_RADIUS = 62.5; //in mm

        CONTROLLER_PORT = 0;

        GEAR_RATIO_LOW = 4.6;
        GEAR_RATIO_HIGH = 2.7;

        LEFTID = 10;
        RIGHTID = 20;
    }

    private CANSparkMax left;
    private CANSparkMax right;

    // underlying mechanism
    private DifferentialDrive drive;

    // distance encoders
    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private double leftDistance, rightDistance;

    // driver controller
    private XboxController controller;

    // private float error;

    private float lastSpeed;

    private double xPos, yPos;

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveJoystick(controller));
    }

	/**
	 * @return the yPos
	 */
	public double getyPos() {
		return yPos;
	}

	/**
	 * @param yPos the yPos to set
	 */
	public void setyPos(double yPos) {
		this.yPos = yPos;
	}

	/**
	 * @return the xPos
	 */
	public double getxPos() {
		return xPos;
	}

	/**
	 * @param xPos the xPos to set
	 */
	public void setxPos(double xPos) {
		this.xPos = xPos;
	}

	public Gear getChosenGear() {
		return ChosenGear;
	}

	public void setChosenGear(Gear chosenGear) {
		this.ChosenGear = chosenGear;
	}

	public DriveTrain() {
        super("DriveTrain", 2.0, 0, 0);

        ChosenDrive = DriveMode.ATWOJOY;
        ChosenGear = Gear.SECOND;
        DriveModeChooser = new SendableChooser<>();

        // initialize hardware
        right = new CANSparkMax(RIGHTID, MotorType.kBrushless);
        left = new CANSparkMax(LEFTID, MotorType.kBrushless);

        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        drive = new DifferentialDrive(left, right);

        initializeEncoders();

        left.setInverted(false);
        right.setInverted(false);

        arcadeDrive(0, 0);
        drive.setSafetyEnabled(false);

        xPos = 0;
        yPos = 0;

        controller = new XboxController(CONTROLLER_PORT);
    }

    @Override
    public void log() {

        updateDistance();

        SmartDashboard.putNumber("DriveTrain/Position/Distance", distance);
        SmartDashboard.putNumber("DriveTrain/Position/xPos", getxPos());
        SmartDashboard.putNumber("DriveTrain/Position/yPos", getyPos());

        if (left.getIdleMode() == IdleMode.kCoast) {
            SmartDashboard.putString("DriveTrain/LeftEncoder/Idle Mode", "Coast");
        } else if (left.getIdleMode() == IdleMode.kBrake) {
            SmartDashboard.putString("DriveTrain/LeftEncoder/Idle Mode", "Brake");
        }

        if (right.getIdleMode() == IdleMode.kCoast) {
            SmartDashboard.putString("DriveTrain/RightEncoder/Idle Mode", "Coast");
        } else if (right.getIdleMode() == IdleMode.kBrake) {
            SmartDashboard.putString("DriveTrain/RightEncoder/Idle Mode", "Brake");
        }

        SmartDashboard.putNumber("DriveTrain/LeftEncoder/Ramp Rate", left.getOpenLoopRampRate());
        SmartDashboard.putNumber("DriveTrain/RightEncoder/Ramp Rate", right.getOpenLoopRampRate());

        SmartDashboard.putNumber("DriveTrain/LeftEncoder/Voltage", left.getBusVoltage());
        SmartDashboard.putNumber("DriveTrain/LeftEncoder/Temperature", left.getMotorTemperature());
        SmartDashboard.putNumber("DriveTrain/LeftEncoder/Output", left.getAppliedOutput());

        SmartDashboard.putNumber("DriveTrain/RightEncoder/Voltage", right.getBusVoltage());
        SmartDashboard.putNumber("DriveTrain/RightEncoder/Temperature", right.getMotorTemperature());
        SmartDashboard.putNumber("DriveTrain/RightEncoder/Output", right.getAppliedOutput());

        SmartDashboard.putNumber("DriveTrain/LeftEncoder/Encoder", leftDistance);
        SmartDashboard.putNumber("DriveTrain/RightEncoder/Encoder", rightDistance);

        DriveModeChooser.setDefaultOption("Arcade - Two Joystick", DriveMode.ATWOJOY);
        DriveModeChooser.addOption("Arcade - One Joystick", DriveMode.AONEJOY);
        DriveModeChooser.addOption("Tank", DriveMode.TANK);
        SmartDashboard.putData("DriveTrain/Drive/Choose Drive", DriveModeChooser);

        SmartDashboard.putString("DriveTrain/Drive/Gear", ChosenGear.toString());

        SmartDashboard.putData("DriveTrain/Position/Reset Displacement", new SetPosition(0, 0));

        //SmartDashboard.putData("DriveTrain/Position/TEST ANGLE", new TurnToAngle(0, Robot.getInstance().getNAVX()));

        ChosenDrive = DriveModeChooser.getSelected();
    }

    /**
     * Tank style driving for the DriveTrain.
     *
     * @param left  Speed in range [-1,1]
     * @param right Speed in range [-1,1]
     */
    public void arcadeDrive(double throttle, double rotate) {
        //lastSpeed = (float) throttle;

        if (ChosenGear == Gear.FIRST){
            throttle *= 0.3;
            rotate *= 0.3;
        }
        else if (ChosenGear == Gear.SECOND){
            throttle *= 0.4;
            rotate *= 0.4;
        }
        else if (ChosenGear == Gear.THIRD){
            throttle *= 0.5;
            rotate *= 0.5;
        }
        
        //throttle = RobotMath.MapJoyValues(throttle, 0.14, 0.4);
        
        //rotate = RobotMath.MapJoyValues(rotate, 0.14, 0.4);
        
        SmartDashboard.putNumber("DriveTrain/Drive/ArcadeDrive/Throttle", throttle);

        SmartDashboard.putNumber("DriveTrain/Drive/ArcadeDrive/Rotate", rotate);

        drive.arcadeDrive(throttle,rotate);
    }

    public void tankdrive(double left, double right) {

        if (ChosenGear == Gear.FIRST){
            left *= 0.3;
            right *= 0.3;
        }
        else if (ChosenGear == Gear.SECOND){
            left *= 0.4;
            right *= 0.4;
        }
        else if (ChosenGear == Gear.THIRD){
            left *= 0.5;
            right *= 0.5;
        }
        
        //left = RobotMath.MapJoyValues(left, 0.14, 0.4);
        
        //right = RobotMath.MapJoyValues(right, 0.14, 0.4);

        SmartDashboard.putNumber("DriveTrain/Drive/TankDrive/Left", left);

        SmartDashboard.putNumber("DriveTrain/Drive/TankDrive/Right", right);

        

        drive.tankDrive(left,right);
    }

    /**
     * Tank style driving for the DriveTrain.
     *
     * @param xbox The XboxController use to drive tank style.
     */
    public void drive(XboxController xbox) {
        // drive(-xbox.getY(GenericHID.Hand.kLeft), xbox.getY(GenericHID.Hand.kRight));
        // drive(-xbox.LeftStick.Y, -xbox.RightStick.Y); //tank drive

        if (ChosenDrive == DriveMode.AONEJOY){
            arcadeDrive(xbox.RightStick.Y, -xbox.RightStick.X);
        }
        else if (ChosenDrive == DriveMode.ATWOJOY){
            arcadeDrive(xbox.LeftStick.Y, -xbox.RightStick.X);
        }
        else if (ChosenDrive == DriveMode.TANK){
            tankdrive(xbox.LeftStick.Y, xbox.RightStick.Y);
        }
    }

    /**
     * Reset the robots sensors to the zero states.
     */
    public void reset() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        setChosenGear(Gear.FIRST);
    }

    public void resetDisplacement(){
        xPos = 0;
        yPos = 0;
    }

    /**
     * Get the average distance of the encoders since the last reset.
     *
     * @return The distance driven (average of leftControllerand
     *         rightControllerencoders).
     */
    
    public double getDistance() {
        return distance;
    }

    private void updateDistance(){
        double changeinDistance = 0;
        double prevDistance = distance;
        leftDistance = -leftEncoder.getPosition();
        rightDistance = rightEncoder.getPosition();
        distance = (leftDistance + rightDistance) / 2;

        AHRS navx = Robot.getInstance().getNAVX();

        double angle = navx.getYaw();

        if (angle >= 0 && angle <= 90){
            angle = RobotMath.mapDouble(angle, 0, 90, 90, 0);
        }
        else if (angle >= 90 && angle <= 180){
            angle = RobotMath.mapDouble(angle, 90, 180, 360, 270);
        }
        else if (angle <= 0 && angle >= -90){
            angle = RobotMath.mapDouble(angle, -90, 0, 180, 90);
        }
        else if (angle <= -90 && angle >= -180){
            angle = RobotMath.mapDouble(angle, -180, -90, 270, 180);
        }

        changeinDistance = distance - prevDistance;

        angle = Math.toRadians(angle);

        setxPos(getxPos() + (changeinDistance * Math.cos(angle)));

        setyPos(getyPos() + (changeinDistance * Math.sin(angle)));
    }

    // will be replace by gyro!

    public double getTurnDistance() {
        return (Math.abs(leftEncoder.getPosition()) + Math.abs(rightEncoder.getPosition())) / 2;
    }

    public CANSparkMax getLeft(){
        return left;
    }

    public CANSparkMax getRight(){
        return right;
    }

    private void initializeEncoders() {
        leftEncoder = left.getEncoder();

        rightEncoder = right.getEncoder();

        leftEncoder.setPositionConversionFactor(0.09); //0.0869565217
        rightEncoder.setPositionConversionFactor(0.09); //0.0869565217
        // final double encoderLinearDistancePerPulse =
        // RobotMath.getDistancePerPulse(COUNTS_PER_REVOLUTION, WHEEL_RADIUS);

        // leftEncoder.setDistancePerPulse(encoderLinearDistancePerPulse);
        // rightEncoder.setDistancePerPulse(encoderLinearDistancePerPulse);
    }

     
    

    @Override
    public void refresh() {
        controller.refresh();
        
        controller.Buttons.X.runCommand(new ChangeGear(1), XboxController.CommandState.WhenPressed);
        controller.Buttons.Y.runCommand(new ChangeGear(2), XboxController.CommandState.WhenPressed);
        controller.Buttons.B.runCommand(new ChangeGear(3), XboxController.CommandState.WhenPressed);
        controller.Buttons.A.runCommand(new ChangeGear(4), XboxController.CommandState.WhenPressed);
        //controller.Buttons.B.runCommand(new TurnToAngle(setPoint, Robot.getInstance().getNAVX()), XboxController.CommandState.WhenPressed);
    }

    @Override
    protected double returnPIDInput() {
        return (rightEncoder.getPosition() - leftEncoder.getPosition());
    }

    @Override
    protected void usePIDOutput(double output) {
        if (output > 0) {
            // too much
            lastSpeed += 0.01;
            drive.tankDrive(lastSpeed, lastSpeed);

        } else if (output < 0) {
            lastSpeed -= 0.01;
            drive.tankDrive(lastSpeed, lastSpeed);
            // too little
        }
        // debug output
        System.out.println("OUTPUT -> " + output);
    }
}
