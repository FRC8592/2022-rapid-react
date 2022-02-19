package frc.robot;

import edu.wpi.first.wpilibj.Timer;


public class Autonomous {
    private enum AutoState{SHOOT, DRIVE, STOP};
    private AutoState currentState = AutoState.SHOOT;
    private Timer autonomousTimer;
    private driveTrain drive;
    private Collector collector;
    
    public Autonomous(driveTrain drive) {
        this.drive = drive;
        autonomousTimer = new Timer();

        autonomousTimer.start();

    }

    public void autoPeriodic() {
        switch(currentState) {
            case SHOOT:                   //shoots ball for 6 seconds
                
                if (autonomousTimer.get() >= 6){       //after 6 seconds we stop shooting and start driving
                    autonomousTimer.reset();
                    currentState = AutoState.DRIVE;
                }
                break;

            case DRIVE:                  //drives robot forward for 1 sec
                //drive.autoDrive();
                if (autonomousTimer.get() >= 1){       //after 1 second we stop
                    autonomousTimer.reset();
                    currentState = AutoState.STOP;
                }
                break;
            
            case STOP:                  //autonomous finished
                //drive.driveStop();
                break;
        }

    }


    public void ballControl(boolean IR1, boolean IR2){
        //Note: Ideally this would be in a switch statement but I haven't quite figured out how to do this yet. Maybe throught the use of an array or list??
        boolean oneBallBottom = IR1 && !IR2;
        boolean oneBallTop = !IR1 && IR2;
        boolean twoBallsCollected = IR1 && IR2;
        boolean noBallsCollected = !IR1 && !IR2;
        
            if(noBallsCollected){
                collector.driveProcessingWheels(); //if we don't have any balls we would like to run the processing wheels
            }else if(oneBallBottom){
                while(oneBallBottom || noBallsCollected){   //WARNING: WHILE LOOPS ARE BAD FIX LATER
                    collector.driveStagingWheels(); //if we go between the two motors we might go into an no balls condition meaning that in this specific if we want to run both when the ball is at 1 ball bottom and while we dont have any balls
                }
            }else if(oneBallTop){
                collector.driveProcessingWheels(); //We want to run the staging wheels while the ball is at the top of the collector
            }else{
                collector.intakeAllStop(); //This is the only option left which is 2 balls collected unless there is manual control going on
            }
    }
}
