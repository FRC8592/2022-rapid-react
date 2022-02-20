package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Collector {

    Collector collector;
    WPI_TalonFX processing;
    WPI_TalonFX staging;

    //This class will contain all new collector specific methods
    public Collector(){
        WPI_TalonFX processing = new WPI_TalonFX(config_hw.newFlywheelCollector);
        WPI_TalonFX staging = new WPI_TalonFX(config_hw.newFlywheelStaging);
        collector = new Collector();
    }

    //Drives the processing wheels for state machine
    public void driveProcessingWheels(){
        processing.set(0.5);
    }

    //Drives staging wheel for state machine
    public void driveStagingWheels(){
        processing.set(0.5);
    }

    //Stops processing wheels for state machine
    public void stopProcessingWheels(){
        processing.set(0);
    }

    //Stops Staging wheels for state machine
    public void stopStagingWheels(){
        processing.set(0);
    }

    //Manually reverses the Staging wheels
    public void reverseStagingWheels(){
        staging.set(-1);
    }

    //Manually reverses the Processing wheels
    public void reverseProcessingWheels(){
        processing.set(-1);
    }

    //Stops entire intake system if needed
    public void intakeAllStop(){
        collector.stopProcessingWheels();
        collector.stopStagingWheels();
    }

    //This runs the intake system in its entirety if needed
    public void intakeAllRun(){
        collector.driveProcessingWheels();
        collector.driveStagingWheels();
    }

    //reverses intake entirely in the event we need it
    public void intakeReverse(){
        collector.reverseProcessingWheels();
        collector.reverseStagingWheels();
    }

    

}
