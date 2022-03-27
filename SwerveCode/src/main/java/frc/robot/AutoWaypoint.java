package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Collector.CollectorState;

public class AutoWaypoint {
    private ArrayList<Waypoint> waypoints;
    private Waypoint currentWaypoint;
    private AutoDrive autoDrive;
    private Drivetrain drivetrain;
    private Collector collector;
    private Shooter shooter;
    private Vision ringVision;
    private Vision ballVision;
    private CollectorState collectorState;

    public AutoWaypoint(AutoDrive locality, Drivetrain drivetrain, Collector collector, Shooter shooter,
            Vision ringVision, Vision ballVision) {
        waypoints = new ArrayList<Waypoint>();
        this.drivetrain = drivetrain;
        this.collector = collector;
        this.shooter = shooter;
        this.ringVision = ringVision;
        this.autoDrive = locality;
        this.ballVision = ballVision;
    }

    public void addWaypoint(Waypoint waypoint) {
        waypoints.add(waypoint);
    }

    public void runWaypoint() {
    //    System.out.println("RUNNING!");
        if (currentWaypoint != null && !currentWaypoint.done) {
            if (currentWaypoint.firstLock){
                System.out.println("TARGETING!");
                drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,
                ringVision.turnRobot(), drivetrain.getGyroscopeRotation()));

                if(ringVision.targetLocked){
                    currentWaypoint.done = true;
                    System.out.println("LOCKED");
                }
            }
            else if (!currentWaypoint.here) {
                System.out.println("Not Here!");
                double turnSpeed = 0;
                if (currentWaypoint.turnTo) {
                    turnSpeed = 0;
                    if(currentWaypoint.fetch){
                        turnSpeed = autoDrive.turnTo(autoDrive.inverseHeading(currentWaypoint.x, currentWaypoint.y),
                        drivetrain.getGyroscopeRotation().getDegrees());
                    }
                    else{
                        turnSpeed = autoDrive.turnTo(autoDrive.getHeading(currentWaypoint.x, currentWaypoint.y),
                        drivetrain.getGyroscopeRotation().getDegrees());
                    }
                        
                } else {
                    turnSpeed = 0;
                }
                double distance = autoDrive.getDistance(currentWaypoint.x, currentWaypoint.y);
                currentWaypoint.here = distance <= currentWaypoint.acceptRadius;
                collectorState = collector.getCollectorState();
                double[] velocity = autoDrive.moveTo(currentWaypoint.x, currentWaypoint.y);
                drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(velocity[0], velocity[1], turnSpeed,
                        drivetrain.getGyroscopeRotation()));
            } else if (currentWaypoint.fetch) {
                //System.out.println("ROBOT fetch!");
                //drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(ballVision.moveTowardsTarget(), 0,
                //ballVision.turnRobot(), Rotation2d.fromDegrees(0)));
                //currentWaypoint.fetch = collector.getCollectorState() == collectorState;
               // System.out.println("ROBOT fetch!: " + currentWaypoint.fetch);
               currentWaypoint.fetch = false;

            } else if (currentWaypoint.shoot) {

                    if (collectorState == CollectorState.NO_BALLS_LOADED) {
                        currentWaypoint.done = true;
                    }
                    if(!ringVision.targetLocked){
                    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,
                    ringVision.turnRobot(), drivetrain.getGyroscopeRotation()));
                    }
                    else {
                        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,
                        0, drivetrain.getGyroscopeRotation()));
                        collector.shoot();
                    }
                    

            } else {
                currentWaypoint.done = true;
            }

        } else if (!waypoints.isEmpty()) {
            currentWaypoint = waypoints.get(0);
            System.out.println("AutoWaypoint New Waypoint");
            if (currentWaypoint.turnTo) {
                currentWaypoint.heading = autoDrive.getHeading(currentWaypoint.x, currentWaypoint.y);
                collectorState = collector.getCollectorState();
            }
            waypoints.remove(0);

        } else {

            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drivetrain.getGyroscopeRotation()));
            return;
        }
    }
}
