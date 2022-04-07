package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Collector.CollectorState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Rotation2d;


public class Autonomous{

    private enum AutoState {
       START, FETCH_A, SHOOT_A, TURN_AWAY_FROM_A, MOVE_A_TO_G, FETCH_D, MOVE_D_TO_G, FETCH_B, SHOOT_B, MOVE_B_TO_G, MOVE_CLOSER_D, SHOOT_D, MOVE_TO_G, FETCH_G, MOVE_CLOSER_G, SHOOT_G, FETCH_C, STOP, FINAL_SHOOT 
      };
    
    private enum FieldLocation{
      START_A, START_B, START_C,
    };

    private FieldLocation startingPosition;

    public Timer timer;
    
    AutoState autoState = AutoState.START;

    // Variables for simple autonomous
    private double autoStateTime;
    private boolean aimLock = false;
    private double lockTime;
    private boolean sweepPositive = true;

public Autonomous() {
    timer = new Timer();
    timer.start();

    

    autoState = AutoState.START;
  }

  public void resetAuto(){
    autoState = AutoState.START;
  }

  public boolean shoot(Drivetrain drive, Collector collector, Vision visionRing, double visonSearchSpeed){
    boolean isDoneShooting = false;
    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, visionRing.turnRobot(visonSearchSpeed), drive.getGyroscopeRotation()));
      collector.shoot();
  

    if(collector.getCollectorState() ==  CollectorState.NO_BALLS_LOADED){
      isDoneShooting = true;
    }else{
      isDoneShooting = false;
    }

    return isDoneShooting;
  }
  
  public void autonomousPeriodic(Vision visionBall, Vision visionRing, CollectorArmMM arm, AutoDrive locality, Collector collector, Shooter shooter, Power powerMonitor, Drivetrain drive) {
    SmartDashboard.putString("Auto State", autoState.toString());
    visionBall.updateVision();
    visionRing.updateVision();
    locality.updatePosition(drive.getYaw(), visionRing);
    arm.update();
    collector.ballControl(arm, shooter, visionRing, powerMonitor);
    shooter.computeFlywheelRPM(visionRing.distanceToTarget(), true, false);
    powerMonitor.powerPeriodic();
    SmartDashboard.putNumber("Auto Timer", timer.get());
    // Turn to ring, then shoot, then drive backwards until we see the ring being 13
    // feet away
    // decide state changes

      switch (autoState) {
        //determine location in field (starting position)
        //locking onto ring, based on angle that robot is facing, it will determine the robot's location
      case START:
        collector.enableCollectMode(arm, powerMonitor);
        SmartDashboard.putBoolean("Gyroscope Rotating?", drive.isGyroscopeRotating());
        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,
          visionRing.turnRobot(ConfigRun.VISION_SEARCH_SPEED), drive.getGyroscopeRotation()));

        if (visionRing.isTargetLocked()){
          if (Math.abs(drive.getAutoHeading() - (Constants.ANGLE_A)) <= Constants.POSITION_ERROR){
            startingPosition = FieldLocation.START_A;
            autoState = AutoState.FETCH_A;
          }
           else if (Math.abs(drive.getAutoHeading() - (Constants.ANGLE_B)) <= Constants.POSITION_ERROR){
            startingPosition = FieldLocation.START_B;
            autoState = AutoState.FETCH_B;
          }
          else if (Math.abs(drive.getAutoHeading() - (Constants.ANGLE_C)) <= Constants.POSITION_ERROR){
            startingPosition = FieldLocation.START_C;
            autoState = AutoState.FETCH_C;
          }
        }
      break;

       // fetch A-ball slowly
       case FETCH_A:
        if(this.fetch(drive, collector, visionBall, -2, -2, CollectorState.TWO_BALLS, -ConfigRun.VISION_SEARCH_SPEED)){
          autoState = AutoState.SHOOT_A;
        }
       break;

       // shoot first 2 balls
       case SHOOT_A:
       if(shoot(drive, collector, visionRing, ConfigRun.VISION_SEARCH_SPEED)){
         timer.reset();
         autoState = AutoState.MOVE_A_TO_G;
       } //todo add switch
       break;

       //collector is up, and turn 200 degrees before moving
       case TURN_AWAY_FROM_A:
        //if(this.turnTo(20, drive, locality)){
          //autoState = AutoState.MOVE_A_TO_G;
        //}
       break;

       //move robot to D-ball
       case MOVE_A_TO_G:
       collector.enableCollectMode(arm, powerMonitor);
      // turnTo(0, drive, locality);
       if(timer.get() >= 1.5){
          this.stopDrive(drive);
         autoState = AutoState.FETCH_G;
       } else {
         drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 2, 0, Rotation2d.fromDegrees(0)));
       }
       break;

       //collect D, slowly
       case FETCH_D:
        if(this.fetch(drive, collector, visionBall, ConfigRun.TARGET_LOCKED_SPEED, ConfigRun.TARGET_CLOSE_SPEED, CollectorState.TWO_BALLS, -ConfigRun.VISION_SEARCH_SPEED)){
          autoState = AutoState.MOVE_CLOSER_D;
        }
       break;

       //use moveto() to move robot to no-man's land
       // G-ball = Gavin's ball
       case MOVE_D_TO_G:
       if(this.driveTo(0, 0, true, locality, drive)){ //edit values when testing
          autoState = AutoState.FETCH_G;
       }
       break;

       // fetch B-ball or C-ball
       case FETCH_B:
        if(this.fetch(drive, collector, visionBall, ConfigRun.TARGET_LOCKED_SPEED, ConfigRun.TARGET_CLOSE_SPEED, CollectorState.TWO_BALLS, -ConfigRun.VISION_SEARCH_SPEED)){
          autoState = AutoState.SHOOT_B;
        }
       break;

       //shoot 2 balls
       case SHOOT_B:
        if(shoot(drive, collector, visionRing, -ConfigRun.VISION_SEARCH_SPEED)){
          timer.reset();
          autoState = AutoState.FETCH_G;
        } //todo add switch
        //? why do we need a switch
       break;

       //move robot to D-ball
       case MOVE_B_TO_G:
        collector.enableCollectMode(arm, powerMonitor);
       if(timer.get() > 1){
          this.stopDrive(drive);
         autoState = AutoState.FETCH_D;
       } else {
         drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-1, 0, 0, Rotation2d.fromDegrees(0)));
       }

       break;

       //move closer to hub to shoot ball D
       case MOVE_CLOSER_D:
       if(!visionRing.isTargetValid()){
       drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,
          visionRing.turnRobot(ConfigRun.VISION_SEARCH_SPEED), drive.getGyroscopeRotation()));
       }else{
        if(this.moveCloserToRing(drive, visionRing, locality, ConfigRun.TARGET_LOCKED_SPEED, ConfigRun.TARGET_CLOSE_SPEED, ConfigRun.VISION_SEARCH_SPEED, 4)){
          autoState = AutoState.SHOOT_D;
        }
       }
       break;

       //shoot ball D
       case SHOOT_D:
       if(shoot(drive, collector, visionRing, -ConfigRun.VISION_SEARCH_SPEED)){
          timer.reset();
          autoState = AutoState.STOP;
       }
       
        //todo add switch
       //what switch? 
       break;

       case MOVE_TO_G:
        collector.enableCollectMode(arm, powerMonitor);
        if(timer.get() > 1.5){
          this.stopDrive(drive);
         autoState = AutoState.FETCH_G;
       } else {
         drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-1, 0, 0, Rotation2d.fromDegrees(0)));
       }
       break;
       

       //assuming we already see G, collect G
       case FETCH_G:
       collector.enableCollectMode(arm, powerMonitor);
       if(this.fetch(drive, collector, visionBall, ConfigRun.TARGET_LOCKED_SPEED, ConfigRun.TARGET_CLOSE_SPEED, CollectorState.TWO_BALLS, -2)){
          autoState = AutoState.MOVE_CLOSER_G;
       }
       break;

       //move closer to hub to shoot G-ball
       case MOVE_CLOSER_G:
       if(!visionRing.isTargetValid()){
        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,
           visionRing.turnRobot(ConfigRun.VISION_SEARCH_SPEED), drive.getGyroscopeRotation()));
        }else{
         if(this.moveCloserToRing(drive, visionRing, locality, ConfigRun.TARGET_LOCKED_SPEED, ConfigRun.TARGET_CLOSE_SPEED, ConfigRun.VISION_SEARCH_SPEED, 5)){
           autoState = AutoState.SHOOT_G;
         }
        }
        
       break;

       //shoot G-ball
       case SHOOT_G:
       if(shoot(drive, collector, visionRing, -ConfigRun.VISION_SEARCH_SPEED)){
          autoState = AutoState.STOP;
       } //todo add switch
       break;

       //starting in START_C position and collect C-ball
       case FETCH_C:
        if(this.fetch(drive, collector, visionBall, ConfigRun.TARGET_LOCKED_SPEED, ConfigRun.TARGET_CLOSE_SPEED, CollectorState.TWO_BALLS, -1)){
          autoState = AutoState.FINAL_SHOOT;
        }
       break;

       case FINAL_SHOOT:
       if(this.shoot(drive, collector, visionBall, -ConfigRun.VISION_SEARCH_SPEED)){
        autoState = AutoState.STOP;
      }
       break;

       //stop autonomous
       case STOP:
       this.stopDrive(drive);
       break;




       /*
      switch (autoState) {
        case AIM:
          if (!aimLock)
            if (visionRing.targetLocked) {
              aimLock = true;
              lockTime = timer.get();
            }
          else
            if (timer.get() >= (lockTime + 2))
              autoState = AutoState.SHOOT;
          break;
          
        case TURN:
          // if (visionRing.targetLocked) {
          //   autoState = AutoState.DRIVE;
          //   autoStateTime = timer.get() + 1.0;
          // }

          autoState = AutoState.DRIVE;
          autoStateTime = timer.get() + 1.0;

          break;
        case SHOOT:

          break;
        case DRIVE:
          if (collector.getCollectorState() == Collector.CollectorState.TWO_BALLS) {
            autoState = AutoState.AIM;

          }
          break;
      }

      SmartDashboard.putString("autoState", autoState.name());
      /*
      // execute current state
      switch (autoState) {
        case AIM:
          drive.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, visionRing.turnRobot(), drive.getGyroscopeRotation()));
          break;

        case SHOOT:
          collector.shoot();
          drive.drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, visionRing.turnRobot(), drive.getGyroscopeRotation()));
          break;
        case TURN:
          // drive.drive(
          //     ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, visionRing.turnRobot(), drive.getGyroscopeRotation()));
          // collector.enableCollectMode(arm, powerMonitor);
          break;
        case DRIVE:
          drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(visionBall.moveTowardsTarget(), 0.0, visionBall.turnRobot(),
              Rotation2d.fromDegrees(0)));
          collector.enableCollectMode(arm, powerMonitor);
          break;
          */
      }
    }
  

  public boolean fetch(Drivetrain drive, Collector collector, Vision visionBall, double targetLockedSpeed, double targetCloseSpeed, CollectorState collectorState, double visionSearchSpeed){
    boolean isDoneFetch = false;

    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(visionBall.moveTowardsTarget(targetLockedSpeed, targetCloseSpeed), 0,
          visionBall.turnRobot(visionSearchSpeed), Rotation2d.fromDegrees(0)));

    if(collector.getCollectorState() == collectorState){
      isDoneFetch = true;
    }else{
      isDoneFetch = false;
    }

    return isDoneFetch;
  }

  public boolean moveCloserToRing(Drivetrain drive, Vision visionRing, AutoDrive locality, double targetLockedSpeed, double targetCloseSpeed, double visionSearchSpeed, double distanceToShoot){
    boolean isInRange = false;
    if(visionRing.distanceToTarget() >= locality.metersToInches(distanceToShoot) && visionRing.isTargetValid()){
      isInRange = false;

      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-visionRing.moveTowardsTarget(targetLockedSpeed, targetCloseSpeed), 0,
      visionRing.turnRobot(visionSearchSpeed), Rotation2d.fromDegrees(0)));
    }else{
      isInRange = true;
      this.stopDrive(drive);
    }
    return isInRange;
  }

  public boolean driveTo(double xPosition, double yPosition, boolean turnTo, AutoDrive locality, Drivetrain drive){
    boolean isDoneDrive;
    double[] velocity = locality.moveTo(xPosition, yPosition);
    double turnSpeed;
    double distance = locality.getDistance(xPosition, yPosition);

    if(turnTo){
      turnSpeed = locality.turnTo(locality.getHeading(xPosition, yPosition), drive.getGyroscopeRotation().getDegrees());
    } else {
      turnSpeed = 0;
    }

    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(velocity[0], velocity[1], turnSpeed,drive.getGyroscopeRotation()));

    if(distance <= 0.2){
      isDoneDrive = true;
    }else{
      isDoneDrive = false;
    }

    return isDoneDrive;
  }

  public void stopDrive(Drivetrain drive){
    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0,drive.getGyroscopeRotation()));
  }
  
  public double turnTo(double targetAngle, Drivetrain drive, double directionToRotate){
    double turnSpeed = 0;
    double currentAngle = drive.getAutoHeading();
    double angleError = 2;

    if(Math.abs(currentAngle - (targetAngle)) <= angleError){
      this.stopDrive(drive);
      turnSpeed = 0;
    }else if(Math.abs(currentAngle - (targetAngle)) > angleError){
      turnSpeed = 1.5 * directionToRotate;
    }
    
    return turnSpeed;
  }

  public boolean sweepFetchMode(double maxAngle, double sweepStartAngle, double centerAngle, Drivetrain drive, Vision visionBall, Collector collector, CollectorState collectorState, double targetLockedSpeed, double targetCloseSpeed, double visionSearchSpeed, double sweepSpeed){
      boolean isDoneFetch = false;
      
    if(this.sweepPositive){
        if(drive.getAutoHeading() <= maxAngle + centerAngle){
          drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, sweepSpeed, Rotation2d.fromDegrees(0)));
        } else {
          this.sweepPositive = false;
        }
    } else{
        if(drive.getAutoHeading() > maxAngle + centerAngle){
          drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -sweepSpeed, Rotation2d.fromDegrees(0)));
        } else {
          this.sweepPositive = true;
        }
    }
      

    
    
    if(collector.getCollectorState() == collectorState){
      isDoneFetch = true;
    }else{
      isDoneFetch = false;
    }

    return isDoneFetch;

  }
}
