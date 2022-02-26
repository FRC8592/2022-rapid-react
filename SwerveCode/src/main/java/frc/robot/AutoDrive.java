package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class AutoDrive {
    double goalX;
    double goalY;

    double currentX;
    double currentY;

    Drivetrain drivetrain;
    Vision vision;
    Locality locality;
    Timer timer;

    double translateKp;
    double translateKd;
    double maxVelocity;
    double errorX;
    double errorY;
    double lastErrorX;
    double lastErrorY;
    double lastTime;
    double currentTime;
    double lastAngle;
    double changeInAngleError;

    double velocityX;
    double velocityY;

    public AutoDrive(Vision vision, Locality locality, Drivetrain drivetrain, Timer timer){
        this.vision = vision;
        this.locality = locality;
        this.drivetrain = drivetrain;
        this.timer = timer;
    }

    public double deltaX(){ //gets the change in angle over time(seconds)
        currentTime = timer.get(); 
        changeInAngleError = (errorX - lastAngle)/(currentTime - lastTime);
        lastErrorX = errorX; // reset initial angle
        lastTime = currentTime; // reset initial time
        return changeInAngleError;
    }

    public double deltaY(){ //gets the change in angle over time(seconds)
        currentTime = timer.get(); 
        changeInAngleError = (errorY - lastAngle)/(currentTime - lastTime);
        lastErrorY = errorY; // reset initial angle
        lastTime = currentTime; // reset initial time
        return changeInAngleError;
    }
    
    public void drive0(double goalX, double goalY){
        errorX = goalX - currentX;
        errorY = goalY - currentY;
        
        velocityX = errorX * translateKp + translateKd*deltaX();
        velocityY = errorY * translateKp + translateKd*deltaY();
    }

}
