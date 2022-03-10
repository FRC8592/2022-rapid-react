package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Collector {
    // Line break sensors
    private DigitalInput lineSensorTop;
    private DigitalInput lineSensorBottom;
    // Collector motors
    private WPI_TalonFX processing;
    private WPI_TalonFX staging;
    private WPI_TalonFX collectorArm;
    // Collector mode
    private boolean collectorMode = false;
    // Internal state
    public enum CollectorState{ONE_BALL_BOTTOM, ONE_BALL_TOP, TWO_BALLS, NO_BALLS_LOADED}
    public CollectorState collectorState;

    //
    // Intialize hardware
    //
    public Collector(){
        collectorArm = new WPI_TalonFX(Constants.newCollectorArm);
        processing   = new WPI_TalonFX(Constants.newFlywheelCollector);
        staging      = new WPI_TalonFX(Constants.newFlywheelStaging);
        lineSensorBottom = new DigitalInput(Constants.LINE_BREAK_BOTTOM_SENSOR_PORT);
        lineSensorTop    = new DigitalInput(Constants.LINE_BREAK_TOP_SENSOR_PORT);
        
        // Invert these motors so that positive power drives balls inward
        processing.setInverted(true);
        staging.setInverted(true);
    }
    
    public void raiseCollectorArm(){
        this.collectorArm.set(-0.);
    }

    public void lowerCollectorArm(){
        this.collectorArm.set(0.05);
    }

    //Drives the processing wheels for state machine
    public void driveProcessingWheels(double speed){
        this.processing.set(speed);
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
        this.driveProcessingWheels(0.2);
        this.driveStagingWheels(0.2);
    }

    //reverses intake entirely in the event we need it
    public void intakeReverse(){
        this.reverseProcessingWheels();
        this.reverseStagingWheels();
    }


   /**
     * Enable collector mode
     * This will drop the collector into position and activate motors to collect
     * 
     * @return A boolean value indicating if collector mode was successfully activated
     */
    public boolean enableCollectorMode() {
        if (collectorState != CollectorState.TWO_BALLS)
            collectorMode = true;
        else
            collectorMode = false;
            
        return collectorMode;
    }


    /**
     * Disable collector mode
     * This will raise the collector and stop the collector motors
     * 
     * @return Always returns true
     */
    public boolean disableCollectorMode() {
        collectorMode = false;
        return true;
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

        SmartDashboard.putString("Collector State", collectorState.toString());
        return collectorState;
    }

    public void ballControl(){

        //this is a simple state machine controlling what ball wheels run and when
        switch(this.determineCollectorState()){
            case NO_BALLS_LOADED: //when there are no balls loaded we want to run the processing wheels to collect 1 ball
                this.driveProcessingWheels(0.2);
                this.driveStagingWheels(0.2);
                this.lowerCollectorArm(); 
            break;
          
            case ONE_BALL_BOTTOM: //when there is one ball at the bottom we want to stop the wheels pushing it in and start driving the middle wheels
                this.driveProcessingWheels(0.2);
                this.driveStagingWheels(0.2);
            break;
             
            case ONE_BALL_TOP: //when theres one ball at the top we want to make sure that the staging wheels don't move the ball and run the bottom wheels to collect another
                this.stopStagingWheels();
                this.driveProcessingWheels(0.2);
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
