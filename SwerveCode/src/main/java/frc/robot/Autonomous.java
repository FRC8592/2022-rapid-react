package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;


public class Autonomous{

    private enum AutoState {
        START, TURN_TO_A
      };
    
    private enum FieldLocation{
      START_A, START_B, START_C
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

  /**
   * Simple 2-ball autonomous routine
   */
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
        
        autoState = AutoState.;
      }
      break;
      // turn robot towards A-ball
       case TURN_TO_A:
       collector.enableCollectMode(arm, powerMonitor);
       drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(visionBall.moveTowardsTarget(), 0.0, visionBall.turnRobot(),
       Rotation2d.fromDegrees(0)));
       if (collector.getCollectorState() == Collector.CollectorState.TWO_BALLS) {
        autoState = AutoState.SHOOT;
       }
       break;
       // fetch A-ball slowly
       case FETCH_A:
       break;
       // shoot first 2 balls
       case SHOOT_A:
       break;
       //collector is up, and turn 200 degrees before moving
       case TURN_AWAY_FROM_A:
       break;
       //move robot to D-ball
       case MOVE_A_TO_D:
       break;
       //collect D, slowly
       case FETCH_D:
       break;
       //use moveto() to move robot to no-man's land
       // G-ball = Gavin's ball
       case MOVE_D_TO_G:
       break;

       // fetch B-ball or C-ball
       case FETCH_BC:
       break
       //shoot 2 balls
       case SHOOT_BC:
       break;
       //move robot to D-ball
       case MOVE_B_TO_D:
       break;
       //move closer to hub to shoot ball D
       case MOVE_CLOSER_D:
       break;
       //shoot ball D
       case SHOOT_D:
       break;
       //assuming we already see G, collect G
       case FETCH_G:
       break;
       //move closer to hub to shoot G-ball
       case MOVE_CLOSER_G
       break;
       //shoot G-ball
       case SHOOT G:
       break;

       //starting in START_C position and collect C-ball
       case FETCH_C:
       break;
       //stop autonomous
       case STOP:
       break;





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
      }
    }
  }
