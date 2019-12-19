package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import java.util.ArrayList;

public class Pneumatics extends Subsystem {

    final private static int PUMP_ID = 0;

    Compressor c;

    ArrayList<DoubleSolenoid> solenoidList;

    boolean SHIFTSTATE;

    private boolean openState;
    private boolean intakeOpenState;



    public Pneumatics(){

        c = new Compressor(PUMP_ID);

        solenoidList = new ArrayList<>();

    }



    @Override

    protected void initDefaultCommand() {

        c.setClosedLoopControl(true);

    }



    public void start(){

        c.setClosedLoopControl(true);

    }



    public void stop(){

        c.setClosedLoopControl(false);

    }



    public void createSolenoid(int solenoidId, int id1, int id2){

        solenoidList.add(solenoidId, new DoubleSolenoid(id1, id2));

        solenoidList.get(solenoidId).set(DoubleSolenoid.Value.kReverse);

        SHIFTSTATE = false;

    }



    public void solenoidOff(int solenoidId){

        try {

            DoubleSolenoid ds = solenoidList.get(solenoidId);

            if(ds != null){

                ds.set(DoubleSolenoid.Value.kOff);

            }

        } catch(IndexOutOfBoundsException e){

            System.out.println(e);

            return;

        }

    }



    public void solenoidForward(int solenoidId){

        try{

            DoubleSolenoid ds = solenoidList.get(solenoidId);

            if(ds != null){

                ds.set(DoubleSolenoid.Value.kForward);

                SHIFTSTATE = true;

            }

        } catch(IndexOutOfBoundsException e){

            System.out.println(e);

            return;

        }

    }



    public void solenoidReverse(int solenoidId){

        try{

            DoubleSolenoid ds = solenoidList.get(solenoidId);

            //DoubleSolenoid ds = new DoubleSolenoid(2,3);

            if(ds != null){

                ds.set(DoubleSolenoid.Value.kReverse);

                SHIFTSTATE = false;

            }

        } catch(IndexOutOfBoundsException e){

            System.out.println(e);

            return;

        }

    }



    public boolean getShiftState(){

        return SHIFTSTATE;

    }

    public boolean getOpenState() {
        return openState;
    }

    public void setOpenState(boolean openState) {
        this.openState = openState;
    }

    public boolean getIntakeOpenState() {return intakeOpenState;}

    public void setIntakeOpenState(boolean intakeOpenState) {
        this.intakeOpenState = intakeOpenState;
    }
}