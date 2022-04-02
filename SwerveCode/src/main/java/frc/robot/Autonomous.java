package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Collector.CollectorState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;


public class Autonomous{

    private enum AutoState {
       START, TURN_TO_A,FETCH_A,SHOOT_A, TURN_AWAY_FROM_A, MOVE_A_TO_D, FETCH_D, MOVE_D_TO_G, FETCH_B, SHOOT_B, MOVE_B_TO_D, MOVE_CLOSER_D, SHOOT_D, FETCH_G, MOVE_CLOSER_G, SHOOT_G, FETCH_C, STOP, SHOOT_C 
      };
    
    private enum FieldLocation{
      START_A, START_B, START_C,
    };

    

    public Timer timer;
    
    AutoState autoState = AutoState.START;

    // Variables for simple autonomous
    private double autoStateTime;
    private boolean aimLock = false;
    private double lockTime;

public Autonomous() {
    timer.start();


    autoState = AutoState.START;
  }

  public boolean shoot(Drivetrain drive, Collector collector, Vision visionRing){
    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, visionRing.turnRobot(), drive.getGyroscopeRotation()));

    if(!drive.isGyroscopeRotating()){
      collector.shoot();
    }

    return collector.getCollectorState() == CollectorState.NO_BALLS_LOADED;
  }
  
  public void autonomousPeriodic(Vision visionBall, Vision visionRing, CollectorArmMM arm, AutoDrive locality, Collector collector, Shooter shooter, Power powerMonitor, Drivetrain drive) {
    
    visionBall.updateVision();
    visionRing.updateVision();
    locality.updatePosition(drive.getYaw(), visionRing);
    arm.update();
    collector.ballControl(arm, shooter, visionRing, powerMonitor);
    shooter.computeFlywheelRPM(visionRing.distanceToTarget(), true, false);
    powerMonitor.powerPeriodic();

   
    // Turn to ring, then shoot, then drive backwards until we see the ring being 13
    // feet away
    // decide state changes

      switch (autoState) {
        //determine location in field (starting position)
        //locking onto ring, based on angle that robot is facing, it will determine the robot's location
        case START:
        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, visionRing.turnRobot(), drive.getGyroscopeRotation()));
      if (!drive.isGyroscopeRotating()){

        
      }
      break;

      // turn robot towards A-ball
       case TURN_TO_A:
       collector.enableCollectMode(arm, powerMonitor);
       this.fetch(drive, collector, visionBall);

       /*
       if (collector.getCollectorState() == Collector.CollectorState.TWO_BALLS) {  //Don't quite know why we need this since we have a shoot state for the bigger overall state machine and we have a shoot method  
       // autoState = AutoState.SHOOT; This 
       }
       */
       break;

       // fetch A-ball slowly
       case FETCH_A:
        if(this.fetch(drive, collector, visionBall)){
          autoState = AutoState.SHOOT_A;
        }
       break;
       // shoot first 2 balls
       case SHOOT_A:
       if(shoot(drive, collector, visionRing)){
         autoState = AutoState.TURN_AWAY_FROM_A;
       } //todo add switch
      
       break;

       //collector is up, and turn 200 degrees before moving
       case TURN_AWAY_FROM_A:
        if(this.turnTo(200, drive, locality)){
          autoState = AutoState.MOVE_A_TO_D;
        }
       break;

       //move robot to D-ball
       case MOVE_A_TO_D:
        if(this.driveTo(0, 0, true, locality, drive)){ //edit values when testing
          autoState = AutoState.FETCH_D;
        }
       break;

       //collect D, slowly
       case FETCH_D:
        if(this.fetch(drive, collector, visionBall)){
          autoState = AutoState.MOVE_D_TO_G;
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
        this.fetch(drive, collector, visionBall);
       break;

       //shoot 2 balls
       case SHOOT_B:
        if(shoot(drive, collector, visionRing)){
          autoState = AutoState.MOVE_B_TO_D;
        } //todo add switch
        //? why do we need a switch
       break;

       //move robot to D-ball
       case MOVE_B_TO_D:
       if(this.driveTo(0, 0, true, locality, drive)){
         autoState = AutoState.MOVE_CLOSER_D;
       }

       break;

       //move closer to hub to shoot ball D
       case MOVE_CLOSER_D:
        if(this.moveCloserToRing(drive, visionRing, locality)){
          autoState = AutoState.SHOOT_D;
        }
       break;

       //shoot ball D
       case SHOOT_D:
       if(shoot(drive, collector, visionRing)){
          autoState = AutoState.FETCH_G;
       } //todo add switch
       //what switch? 
       break;

       //assuming we already see G, collect G
       case FETCH_G:
       if(this.fetch(drive, collector, visionBall)){
          autoState = AutoState.MOVE_CLOSER_G;
       }
       break;

       //move closer to hub to shoot G-ball
       case MOVE_CLOSER_G:
        if(this.moveCloserToRing(drive, visionRing, locality)){
          autoState = AutoState.SHOOT_G;
        }
       break;

       //shoot G-ball
       case SHOOT_G:
       if(shoot(drive, collector, visionRing)){
          autoState = AutoState.STOP;
       } //todo add switch
       break;

       //starting in START_C position and collect C-ball
       case FETCH_C:
        if(this.fetch(drive, collector, visionBall)){
          autoState = AutoState.SHOOT_C;
        }
       break;

       case SHOOT_C:
       if(this.shoot(drive, collector, visionBall)){
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
  

  public boolean fetch(Drivetrain drive, Collector collector, Vision visionBall){
    boolean isDoneFetch = false;
    CollectorState collectorState = collector.getCollectorState();

    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(visionBall.moveTowardsTarget(), 0,
          visionBall.turnRobot(), Rotation2d.fromDegrees(0)));

    if(collector.getCollectorState() == collectorState){
      isDoneFetch = true;
    }else{
      isDoneFetch = false;
    }

    return isDoneFetch;
  }

  public boolean moveCloserToRing(Drivetrain drive, Vision visionRing, AutoDrive locality){
    boolean isInRange = false;
    if(visionRing.distanceToTarget() >= locality.metersToInches(3)){
      isInRange = false;

      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(visionRing.moveTowardsTarget(), 0,
      visionRing.turnRobot(), Rotation2d.fromDegrees(0)));
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
  
  public boolean turnTo(double angle, Drivetrain drive, AutoDrive locality){
    boolean isDoneTurn = false;
    double turnSpeed = locality.turnTo(angle, drive.getGyroscopeRotation().getDegrees());

    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turnSpeed, drive.getGyroscopeRotation()));

    if(drive.getGyroscopeRotation().getDegrees() != angle){
      isDoneTurn = false;
    }else{
      isDoneTurn = true;
    }

    return isDoneTurn;
  }
}
