package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Collector {
    // Control variables
    private boolean unjamMode     = false;
    private boolean shootMode     = false;
    private boolean collectorMode = false;

    // Line break sensors
    private DigitalInput lineSensorTop;
    private DigitalInput lineSensorBottom;

    // Collector motors
    private WPI_TalonFX processing;
    private WPI_TalonFX staging;

    // Internal state
    public enum CollectorState{NO_BALLS_LOADED, ONE_BALL_BOTTOM, BALL_XFER_TO_TOP, ONE_BALL_TOP, TWO_BALLS}
    public CollectorState collectorState = CollectorState.NO_BALLS_LOADED;

    public Vision vision;


    //
    // Intialize hardware
    //
    public Collector() {
        processing   = new WPI_TalonFX(Constants.newFlywheelCollector);
        staging      = new WPI_TalonFX(Constants.newFlywheelStaging);
        lineSensorBottom = new DigitalInput(Constants.LINE_BREAK_BOTTOM_SENSOR_PORT);
        lineSensorTop    = new DigitalInput(Constants.LINE_BREAK_TOP_SENSOR_PORT);
        
        // Invert collector motors so that positive power drives balls inward
        processing.setInverted(true);
        staging.setInverted(true);
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
    public boolean enableCollectMode(CollectorArmPID arm, Power powerMonitor) {
        if (collectorState != CollectorState.TWO_BALLS) {
            collectorMode = true;
            arm.lowerArm();
            powerMonitor.relayOn(); // Turn on light
        } else
            collectorMode = false;
            
        return collectorMode;
    }


    /**
     * Disable collector mode
     * This will raise the collector and stop the collector motors
     * 
     * @return Always returns true
     */
    public boolean disableCollectMode(CollectorArmPID arm, Power powerMonitor) {
        collectorMode = false;
        arm.raiseArm();
        powerMonitor.relayOff();    // Turn off light

        return true;
    }


    /**
     * Raise the collector arm and run motors in reverse
     * *
     * @return
     */
    public void unjam(CollectorArmPID arm) {
        arm.raiseArm();
        unjamMode = true;
    }


    /**
     * Activate shooting mode, if a ball is available in the upper position
     */
    public void shoot() {
        if (!lineSensorTop.get())   // False indcates that a ball is present
            shootMode = true;
    }


    /**
     * Control in which state the collector is operating
     * 
     * @return The state of the collector encoded as CollectorState
     */
    public CollectorState determineCollectorState(){
        boolean topState    = lineSensorTop.get();
        boolean bottomState = lineSensorBottom.get();

        // Control which state we are in
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

        // SmartDashboard.getBoolean("LineSensorTop", topState);
        // SmartDashboard.getBoolean("LineSensorBottom", bottomState);
        // SmartDashboard.putString("Collector State", collectorState.toString());

        return collectorState;
    }


    /**
     * Control collector mechanisms based on operating state
     */
    public void ballControl(CollectorArmPID arm, Power powerMonitor, Shooter shooter) {
        boolean topBall    = !lineSensorTop.get();
        boolean bottomBall = !lineSensorBottom.get();

        SmartDashboard.getBoolean("LineSensorTop", topBall);
        SmartDashboard.getBoolean("LineSensorBottom", bottomBall);
        SmartDashboard.putString("Collector State", collectorState.toString());

        //
        // *** Unjam overrides any normal control ***
        //
        if (unjamMode) {
            driveProcessingWheels(Constants.UNJAM_PROCESSING_POWER);
            driveStagingWheels(Constants.UNJAM_STAGING_POWER);
            unjamMode = false;  // Clear mode.  Will be overwritten if unjam button is held down
        }
        //
        // Shoot mode overrides normal loading operations
        //
        else if (shootMode) {
            if (shooter.isFlywheelReady() & vision.isTargetLocked()) {
                driveProcessingWheels(0);
                driveStagingWheels(Constants.SHOOT_STAGING_POWER);
            }

            // Stop shoot mode once the top ball is gone (true means no ball is present)
            if (lineSensorTop.get())
                shootMode = false;
        }
        
        //
        // Process the normal state machine
        //
        else {
            //this is a simple state machine controlling what ball wheels run and when
            switch(collectorState) {

                case NO_BALLS_LOADED: //when there are no balls loaded we want to run the processing wheels to collect 1 ball
                    if (topBall && bottomBall)
                        collectorState = CollectorState.TWO_BALLS;
                    else if (topBall)
                        collectorState = CollectorState.ONE_BALL_TOP;
                    else if (bottomBall)
                        collectorState = CollectorState.ONE_BALL_BOTTOM;


                    if (collectorMode) {
                        driveProcessingWheels(Constants.COLLECT_PROCESSING_POWER);
                        driveStagingWheels(Constants.COLLECT_STAGING_POWER);
                    }
                    else {
                        driveProcessingWheels(0);
                        driveStagingWheels(0);   
                    }
                break;
            
                case ONE_BALL_BOTTOM: //when there is one ball at the bottom we want to stop the wheels pushing it in and start driving the middle wheels
                    if (!topBall & !bottomBall)
                        collectorState = CollectorState.BALL_XFER_TO_TOP;
                    else if (topBall & bottomBall)
                        collectorState = CollectorState.TWO_BALLS;
                    else if (topBall)
                        collectorState = CollectorState.ONE_BALL_TOP;

                    //
                    // We don't check for collectorMode here, because if we have one ball in the bottom, we always want to move it into the
                    // the top position
                    //
                    driveProcessingWheels(Constants.COLLECT_PROCESSING_POWER);
                    driveStagingWheels(Constants.COLLECT_STAGING_POWER);
                break;

                case BALL_XFER_TO_TOP:
                    if (topBall && bottomBall)
                        collectorState = CollectorState.TWO_BALLS;
                    else if (topBall)
                        collectorState = CollectorState.ONE_BALL_TOP;
                    else if (bottomBall)
                        collectorState = CollectorState.ONE_BALL_BOTTOM;

                    //
                    // We don't check for collectorMode here, because if we have one ball in the bottom, we always want to move it into the
                    // the top position
                    //

                    driveProcessingWheels(Constants.COLLECT_PROCESSING_POWER);
                    driveStagingWheels(Constants.COLLECT_STAGING_POWER);
                break;

                case ONE_BALL_TOP: //when theres one ball at the top we want to make sure that the staging wheels don't move the ball and run the bottom wheels to collect another
                    if (topBall && bottomBall)
                        collectorState = CollectorState.TWO_BALLS;
                    else if (topBall)
                        collectorState = CollectorState.ONE_BALL_TOP;
                    else if (bottomBall)
                        collectorState = CollectorState.ONE_BALL_BOTTOM;
                    else
                        collectorState = CollectorState.NO_BALLS_LOADED;

                    if (collectorMode) {
                        driveProcessingWheels(Constants.COLLECT_PROCESSING_POWER);
                        driveStagingWheels(0);
                    }
                    else {
                        driveProcessingWheels(0);
                        driveStagingWheels(0);
                    }
                break;

                case TWO_BALLS: //when we have 2 balls we don't want to run any of the intake modules
                    if (!topBall && !bottomBall)    // We just shot and the bottom ball is on its way up (i.e. between sensors)
                        collectorState = CollectorState.BALL_XFER_TO_TOP;
                    else if (topBall && !bottomBall)
                        collectorState = CollectorState.ONE_BALL_TOP;
                    else if (bottomBall && !topBall)
                        collectorState = CollectorState.ONE_BALL_BOTTOM;
                    
                    driveProcessingWheels(0);
                    driveStagingWheels(0);

                    disableCollectMode(arm, powerMonitor);
                break;
            }

        }   
    }

    public void manualControl(){
            
    }


}
