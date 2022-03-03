package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Collector {

    
    private DigitalInput lineSensorTop;
    private DigitalInput lineSensorBottom;
    WPI_TalonFX processing;
    WPI_TalonFX staging;
    WPI_TalonFX collectorArm;
    public enum CollectorState{ONE_BALL_BOTTOM, ONE_BALL_TOP, TWO_BALLS, NO_BALLS_LOADED}
    private CollectorState collectorState;

    //This class will contain all new collector specific methods
    public Collector(){
        collectorArm = new WPI_TalonFX(Constants.newCollectorArm);
        processing = new WPI_TalonFX(Constants.newFlywheelCollector);
        staging = new WPI_TalonFX(Constants.newFlywheelStaging);
        lineSensorBottom = new DigitalInput(Constants.LINE_BREAK_BOTTOM_SENSOR_PORT);
        lineSensorTop = new DigitalInput(Constants.LINE_BREAK_TOP_SENSOR_PORT);
        
        processing.setInverted(true);
        staging.setInverted(true);
    }
    
    public void raiseCollectorArm(){
        this.collectorArm.set(-0.2);
    }

    public void lowerCollectorArm(){
        this.collectorArm.set(0.2);
    }

    //Drives the processing wheels for state machine
    public void driveProcessingWheels(){
        this.processing.set(0.2);
    }

    //Drives staging wheel for state machine
    public void driveStagingWheels(double speed){
        this.staging.set(speed);
    }

    //Stops processing wheels for state machine
    public void stopProcessingWheels(){
        this.processing.set(0);
    }


    //Stops Staging wheels for state machine
    public void stopStagingWheels(){
        this.staging.set(0);
    }

    //Manually reverses the Staging wheels
    public void reverseStagingWheels(){
        this.staging.set(-0.2);
    }

    //Manually reverses the Processing wheels
    public void reverseProcessingWheels(){
        this.processing.set(-0.2);
    }

    //Stops entire intake system if needed
    public void intakeAllStop(){
        this.stopProcessingWheels();
        this.stopStagingWheels();
    }

    //This runs the intake system in its entirety if needed
    public void intakeAllRun(){
        this.driveProcessingWheels();
        this.driveStagingWheels(0.2);
    }

    //reverses intake entirely in the event we need it
    public void intakeReverse(){
        this.reverseProcessingWheels();
        this.reverseStagingWheels();
    }

    public CollectorState determineCollectorState(){
        boolean topState = lineSensorTop.get();
        SmartDashboard.getBoolean("LineSensorTop", topState);
        boolean bottomState = lineSensorBottom.get();
        SmartDashboard.getBoolean("LineSensorBottom", bottomState);

        //this makes sure to tell which states are which
        if(!bottomState){
            if(!topState){
                collectorState = CollectorState.TWO_BALLS;
            }else{
                collectorState = CollectorState.ONE_BALL_BOTTOM;
            }
        }else{
            if(!topState){
                collectorState = CollectorState.ONE_BALL_TOP;
            }else{
                collectorState = CollectorState.NO_BALLS_LOADED;
            }
        }




        return collectorState;
    }

    public void ballControl(){

        //this is a simple state machine controlling what ball wheels run and when
        switch(this.determineCollectorState()){
            case NO_BALLS_LOADED: //when there are no balls loaded we want to run the processing wheels to collect 1 ball
                this.driveProcessingWheels();
                this.driveStagingWheels(0.2);
                this.lowerCollectorArm(); 
            break;
          
            case ONE_BALL_BOTTOM: //when there is one ball at the bottom we want to stop the wheels pushing it in and start driving the middle wheels
                this.driveProcessingWheels();
                this.driveStagingWheels(0.2);
            break;
             
            case ONE_BALL_TOP: //when theres one ball at the top we want to make sure that the staging wheels don't move the ball and run the bottom wheels to collect another
                this.stopStagingWheels();
                this.driveProcessingWheels();
            break;

            case TWO_BALLS: //when we have 2 balls we don't want to run any of the intake modules
                this.intakeAllStop();
                this.raiseCollectorArm();
            break;
        }   



        SmartDashboard.putBoolean("LineSensorTop", lineSensorTop.get());
        SmartDashboard.putBoolean("LineSensorBottom", lineSensorBottom.get());
    }

    public void manualControl(){
            
    }


}
