package frc.robot;

import edu.wpi.first.wpilibj.Timer;


public class Autonomous {
    private enum AutoState{SHOOT, DRIVE, STOP};
    private AutoState currentState = AutoState.SHOOT;
    private Timer autonomousTimer;
    private driveTrain drive;
    
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
                drive.autoDrive();
                if (autonomousTimer.get() >= 1){       //after 1 second we stop
                    autonomousTimer.reset();
                    currentState = AutoState.STOP;
                }
                break;
            
            case STOP:                  //autonomous finished
                drive.driveStop();
                break;
        }

    }
}
