package frc.robot.subsystem;

import frc.robot.Robot;
import frc.robot.commands.ChangeGear;
import frc.robot.commands.DriveJoystick;
import frc.robot.commands.SetPosition;
import frc.robot.util.Loggable;
import frc.robot.util.RobotMath;
import frc.robot.util.XboxController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;

public class DriveTrain extends PIDSubsystem implements Loggable, Refreshable {

    // Drive Modes
    private enum DriveMode {
        AONEJOY, ATWOJOY, TANK,
    }

    // Gears (Speeds)
    public enum Gear {
        FIRST, SECOND, THIRD, FOURTH,
    }

    // Controlller Port
    private static int CONTROLLER_PORT;

    // Gear Ratios
    public static final double GEAR_RATIO_LOW;
    public static final double GEAR_RATIO_HIGH;

    // SparkMax CANIDs
    private static final int LEFTFID;
    private static final int RIGHTFID;
    private static final int LEFTLID;
    private static final int RIGHTLID;

    private DriveMode ChosenDrive;
    private Gear ChosenGear;
    private SendableChooser<DriveMode> DriveModeChooser;
    private static final double FIRSTGEARRATIO;
    private static final double SECONDGEARRATIO;
    private static final double THIRDGEARRATIO;


    // SparkMax Objects
    private CANSparkMax leftF;
    private CANSparkMax rightF;
    private CANSparkMax leftL;
    private CANSparkMax rightL;

    // underlying mechanism
    private DifferentialDrive drive;

    // Distance Encoders
    private CANEncoder leftLEncoder;
    private CANEncoder rightLEncoder;
    private CANEncoder leftFEncoder;
    private CANEncoder rightFEncoder;

    private double distance, leftDistance, rightDistance;

    // Driver Controller
    private XboxController controller;

    private float lastSpeed;

    private double xPos, yPos;

    static {
        // Initialization
        CONTROLLER_PORT = 0;

        GEAR_RATIO_LOW = 4.6;
        GEAR_RATIO_HIGH = 2.7;

        LEFTLID = 10;
        LEFTFID = 11;
        RIGHTLID = 20;
        RIGHTFID = 21;

        FIRSTGEARRATIO = 0.3;
        SECONDGEARRATIO = 0.4;
        THIRDGEARRATIO = 0.5;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveJoystick(controller));
    }

    public double getyPos() {
        return yPos;
    }

    public void setyPos(double yPos) {
        this.yPos = yPos;
    }

    public double getxPos() {
        return xPos;
    }

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
        rightL = new CANSparkMax(RIGHTLID, MotorType.kBrushless);
        leftL = new CANSparkMax(LEFTLID, MotorType.kBrushless);
        rightF = new CANSparkMax(RIGHTFID, MotorType.kBrushless);
        leftF = new CANSparkMax(LEFTFID, MotorType.kBrushless);

        leftL.restoreFactoryDefaults();
        rightL.restoreFactoryDefaults();
        leftF.restoreFactoryDefaults();
        rightF.restoreFactoryDefaults();

        rightF.follow(rightL);
        leftF.follow(leftL);

        drive = new DifferentialDrive(leftL, rightL);

        initializeEncoders();

        leftL.setInverted(true);
        rightL.setInverted(true);
        leftF.setInverted(true);
        rightF.setInverted(true);

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

        if (leftL.getIdleMode() == IdleMode.kCoast) {
            SmartDashboard.putString("DriveTrain/LeftEncoder/Idle Mode", "Coast");
        } else if (leftL.getIdleMode() == IdleMode.kBrake) {
            SmartDashboard.putString("DriveTrain/LeftEncoder/Idle Mode", "Brake");
        }

        if (rightL.getIdleMode() == IdleMode.kCoast) {
            SmartDashboard.putString("DriveTrain/RightEncoder/Idle Mode", "Coast");
        } else if (rightL.getIdleMode() == IdleMode.kBrake) {
            SmartDashboard.putString("DriveTrain/RightEncoder/Idle Mode", "Brake");
        }

        SmartDashboard.putNumber("DriveTrain/LeftEncoder/Ramp Rate", leftL.getOpenLoopRampRate());
        SmartDashboard.putNumber("DriveTrain/RightEncoder/Ramp Rate", rightL.getOpenLoopRampRate());

        SmartDashboard.putNumber("DriveTrain/LeftEncoder/Voltage", leftL.getBusVoltage());
        SmartDashboard.putNumber("DriveTrain/LeftEncoder/Temperature", leftL.getMotorTemperature());
        SmartDashboard.putNumber("DriveTrain/LeftEncoder/Output", leftL.getAppliedOutput());

        SmartDashboard.putNumber("DriveTrain/RightEncoder/Voltage", rightL.getBusVoltage());
        SmartDashboard.putNumber("DriveTrain/RightEncoder/Temperature", rightL.getMotorTemperature());
        SmartDashboard.putNumber("DriveTrain/RightEncoder/Output", rightL.getAppliedOutput());

        SmartDashboard.putNumber("DriveTrain/LeftEncoder/Encoder", leftDistance);
        SmartDashboard.putNumber("DriveTrain/RightEncoder/Encoder", rightDistance);

        DriveModeChooser.setDefaultOption("Arcade - Two Joystick", DriveMode.ATWOJOY);
        DriveModeChooser.addOption("Arcade - One Joystick", DriveMode.AONEJOY);
        DriveModeChooser.addOption("Tank", DriveMode.TANK);
        SmartDashboard.putData("DriveTrain/Drive/Choose Drive", DriveModeChooser);

        SmartDashboard.putString("DriveTrain/Drive/Gear", ChosenGear.toString());

        SmartDashboard.putData("DriveTrain/Position/Reset Displacement", new SetPosition(0, 0));

        ChosenDrive = DriveModeChooser.getSelected();
    }

    public void arcadeDrive(double throttle, double rotate) {
         lastSpeed = (float) throttle;

        if (ChosenGear == Gear.FIRST) {
            throttle *= FIRSTGEARRATIO;
            rotate *= FIRSTGEARRATIO;
        } else if (ChosenGear == Gear.SECOND) {
            throttle *= SECONDGEARRATIO;
            rotate *= SECONDGEARRATIO;
        } else if (ChosenGear == Gear.THIRD) {
            throttle *= THIRDGEARRATIO;
            rotate *= THIRDGEARRATIO;
        }

        // throttle = RobotMath.MapJoyValues(throttle, 0.14, 0.4);

        // rotate = RobotMath.MapJoyValues(rotate, 0.14, 0.4);

        SmartDashboard.putNumber("DriveTrain/Drive/ArcadeDrive/Throttle", throttle);

        SmartDashboard.putNumber("DriveTrain/Drive/ArcadeDrive/Rotate", rotate);

        drive.arcadeDrive(throttle, rotate);
    }

    public void tankdrive(double left, double right) {

        if (ChosenGear == Gear.FIRST) {
            left *= FIRSTGEARRATIO;
            right *= FIRSTGEARRATIO;
        } else if (ChosenGear == Gear.SECOND) {
            left *= SECONDGEARRATIO;
            right *= SECONDGEARRATIO;
        } else if (ChosenGear == Gear.THIRD) {
            left *= THIRDGEARRATIO;
            right *= THIRDGEARRATIO;
        }

        // left = RobotMath.MapJoyValues(left, 0.14, 0.4);

        // right = RobotMath.MapJoyValues(right, 0.14, 0.4);

        SmartDashboard.putNumber("DriveTrain/Drive/TankDrive/Left", left);

        SmartDashboard.putNumber("DriveTrain/Drive/TankDrive/Right", right);

        drive.tankDrive(left, right);
    }

    public void drive(XboxController xbox) {
        if (ChosenDrive == DriveMode.AONEJOY) {
            arcadeDrive(xbox.RightStick.Y, -xbox.RightStick.X);
        } else if (ChosenDrive == DriveMode.ATWOJOY) {
            arcadeDrive(xbox.LeftStick.Y, -xbox.RightStick.X);
        } else if (ChosenDrive == DriveMode.TANK) {
            tankdrive(xbox.LeftStick.Y, xbox.RightStick.Y);
        }
    }

    /**
     * Reset the robots sensors to the zero states.
     */
    public void reset() {
        leftLEncoder.setPosition(0);
        rightLEncoder.setPosition(0);
        setChosenGear(Gear.SECOND);
    }

    public void resetDisplacement() {
        xPos = 0;
        yPos = 0;
    }

    public double getDistance() {
        return distance;
    }

    private void updateDistance() {
        double changeinDistance = 0;
        double prevDistance = distance;
        leftDistance = -leftLEncoder.getPosition();
        rightDistance = rightLEncoder.getPosition();
        distance = (leftDistance + rightDistance) / 2;

        AHRS navx = Robot.getInstance().getNAVX();

        double angle = navx.getYaw();

        if (angle >= 0 && angle <= 90) {
            angle = RobotMath.mapDouble(angle, 0, 90, 90, 0);
        } else if (angle >= 90 && angle <= 180) {
            angle = RobotMath.mapDouble(angle, 90, 180, 360, 270);
        } else if (angle <= 0 && angle >= -90) {
            angle = RobotMath.mapDouble(angle, -90, 0, 180, 90);
        } else if (angle <= -90 && angle >= -180) {
            angle = RobotMath.mapDouble(angle, -180, -90, 270, 180);
        }

        changeinDistance = distance - prevDistance;

        angle = Math.toRadians(angle);

        setxPos(getxPos() + (changeinDistance * Math.cos(angle)));

        setyPos(getyPos() + (changeinDistance * Math.sin(angle)));
    }

    public CANSparkMax getLeftL() {
        return leftL;
    }

    public CANSparkMax getRightL() {
        return rightL;
    }

    private void initializeEncoders() {
        leftLEncoder = leftL.getEncoder();
        leftFEncoder = leftF.getEncoder();

        rightLEncoder = rightL.getEncoder();
        rightFEncoder = rightF.getEncoder();

        leftLEncoder.setPositionConversionFactor(0.09); // 0.0869565217
        rightLEncoder.setPositionConversionFactor(0.09); // 0.0869565217
        leftFEncoder.setPositionConversionFactor(0.09); // 0.0869565217
        rightFEncoder.setPositionConversionFactor(0.09); // 0.0869565217
    }

    @Override
    public void refresh() {
        controller.refresh();

        controller.Buttons.X.runCommand(new ChangeGear(1), XboxController.CommandState.WhenPressed);
        controller.Buttons.Y.runCommand(new ChangeGear(2), XboxController.CommandState.WhenPressed);
        controller.Buttons.B.runCommand(new ChangeGear(3), XboxController.CommandState.WhenPressed);
        controller.Buttons.A.runCommand(new ChangeGear(4), XboxController.CommandState.WhenPressed);
    }

    @Override
    protected double returnPIDInput() {
        return (rightLEncoder.getPosition() - leftLEncoder.getPosition());
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
