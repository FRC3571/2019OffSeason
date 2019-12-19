/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import ca.team3571.offseason.auto.AutonomousExecutor;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.LiftManualCommand;
import frc.robot.commands.OpenCloseCommand;
import frc.robot.commands.auto.PracticeAuto;
//import ca.team3571.offseason.commands.auto.PracticeAuto;
import frc.robot.component.CameraController;
import frc.robot.component.RobotCamera;
import frc.robot.subsystem.*;
import frc.robot.subsystem.elevator.Elevator;
import frc.robot.subsystem.elevator.LiftCommand;
import frc.robot.util.RioDuino;
import frc.robot.util.RobotMath;
import frc.robot.util.XboxController;
import edu.wpi.cscore.UsbCamera;
//import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
// If you rename or move this class, update the build.properties file in the project root
public class Robot extends TimedRobot //TimedRobot
{

    private DriveTrain driveTrain;
    private Elevator elevator;
    private Intake intake;
    private Tilt tilt;
    private Pneumatics pneumatics;
    private Logger logger;
    private RioDuino rioDuino;
    private CameraController cameraController;
    private XboxController subsystemController = new XboxController(1);
    private PowerDistributionPanel pdp;
    private AHRS navx;
    private static Robot exposedInstance;

    /**
     * Exposes instance once it's ready and populated
     * @return singleton instance of robot
     */
    public static Robot getInstance() {
        if(exposedInstance==null) {
            throw new IllegalStateException("#robotInit must finish its invocation!");
        }
        return exposedInstance;
    }

    public XboxController getSubsystemController() {
        return subsystemController;
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        //set reference for exposed instance
        exposedInstance = this;

        //power distribution
        pdp = new PowerDistributionPanel();

        //initialize subsystems
        pneumatics = new Pneumatics();
        driveTrain = new DriveTrain();
        elevator = new Elevator();
        intake = new Intake();
        tilt = new Tilt();

        //logger
        logger = Logger.getLogger(getClass().getName());

        //rio
        rioDuino = new RioDuino();

        navx = new AHRS(SPI.Port.kMXP); 

        runCamera();
        initController();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
//        boolean signalReceived = false;
//        String signal = null;
//        while(!signalReceived) {
//            signal = DriverStation.getInstance().getGameSpecificMessage();
//            if(signal.length()==AutonomousExecutor.SIGNAL_LENGTH) {
//                signalReceived = true;
//            }
//        }
//        new AutonomousExecutor().accept(signal);
        new PracticeAuto().start();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run(); //dont delete this u idiot
        //teleopPeriodic();
        
        debug();
    }

    /**
     * This function is called periodically during operator control.
     * gets called every 20 millis
     */
    @Override
    public void teleopPeriodic() {
        driveTrain.refresh();
        elevator.refresh();
        intake.refresh();
        tilt.refresh();
        subsystemController.refresh();
        Scheduler.getInstance().run();
       
        debug();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        new Thread() {
            @Override
            public void start() {
                while(true) {
                    System.out.println(rioDuino.receiveData());
                    for(byte b: rioDuino.getRaw()) {
                        System.out.print(" "+(int)b);
                    }
                    try {
                        sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }.start();
    }

    private void runCamera() {
        UsbCamera cameraOne = CameraServer.getInstance().startAutomaticCapture(0);
        UsbCamera cameraTwo = CameraServer.getInstance().startAutomaticCapture(1);
        //UsbCamera cameraThree = CameraServer.getInstance().startAutomaticCapture(2);
        cameraOne.setWhiteBalanceAuto();
        cameraTwo.setWhiteBalanceAuto();
        //cameraThree.setWhiteBalanceAuto();

        cameraController = new CameraController();//new RobotCamera(cameraOne), new RobotCamera(cameraTwo));
        cameraController.addCamera(new RobotCamera(cameraOne));
        cameraController.addCamera(new RobotCamera(cameraTwo));
        //cameraController.addCamera(new RobotCamera(cameraThree));
        cameraController.begin();
        cameraController.setPriority(0);
    }

    private void debug() {
        driveTrain.log();
        elevator.log();
        intake.log();
        tilt.log();
        lognavx();
    }

    private void lognavx(){
        SmartDashboard.putNumber("DriveTrain/Position/Yaw", navx.getYaw());
    }

    public void log(Level logLevel, String message) {
        logger.log(logLevel, message);
    }

    //getters
    public DriveTrain getDrive() {
        return driveTrain;
    }

    public Pneumatics getPneumatics() {
        return pneumatics;
    }

    public Intake getIntake() {
        return intake;
    }

    public Tilt getTilt() {
        return tilt;
    }

    public Elevator getElevator() {
        return elevator;
    }

    public PowerDistributionPanel getPowerData() {
        return pdp;
    }

    public AHRS getNAVX(){
        return navx;
    }

    private void initController() {
     //   subsystemController.Buttons.Y.runCommand(new ClimbCommand(), XboxController.CommandState.WhenPressed);
     //   subsystemController.Buttons.LB.runCommand(new LiftCommand(true), XboxController.CommandState.WhenPressed);
      //  subsystemController.Buttons.RB.runCommand(new LiftCommand(false), XboxController.CommandState.WhenPressed);

        //climbing  
        subsystemController.Buttons.Y.runCommand(new ClimbCommand(), XboxController.CommandState.WhenPressed);
      
        //intake  
        subsystemController.Buttons.A.runCommand(new OpenCloseCommand(), XboxController.CommandState.WhenPressed);
        //subsystemController.Buttons.B.runCommand(new TiltCommand(), XboxController.CommandState.WhenPressed);
        
        //elevator 
        subsystemController.Buttons.LB.runCommand(new LiftCommand(true), XboxController.CommandState.WhenPressed);
        subsystemController.Buttons.RB.runCommand(new LiftCommand(false), XboxController.CommandState.WhenPressed);
    }

}
