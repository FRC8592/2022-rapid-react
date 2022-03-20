package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Collector.CollectorState;

public class AutoWaypoint {
    private ArrayList<Waypoint> waypoints;
    private Waypoint currentWaypoint;
    private AutoDrive autoDrive;
    private Drivetrain drivetrain;
    private Collector collector;
    private Shooter shooter;
    private Vision vision;
    private CollectorState collectorState;

    public AutoWaypoint(AutoDrive locality, Drivetrain drivetrain, Collector collector, Shooter shooter,
            Vision vision) {
        waypoints = new ArrayList<Waypoint>();
        this.drivetrain = drivetrain;
        this.collector = collector;
        this.shooter = shooter;
        this.vision = vision;
    }

    public void addWaypoint(Waypoint waypoint) {
        waypoints.add(waypoint);
    }

    public void runWaypoint() {
        if (currentWaypoint != null && !currentWaypoint.done) {
            if (!currentWaypoint.here) {
                double turnSpeed;
                if (currentWaypoint.turnTo) {
                    turnSpeed = autoDrive.turnTo(autoDrive.getHeading(currentWaypoint.x, currentWaypoint.y),
                            drivetrain.getYaw());
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
                vision.moveTowardsTarget();
                currentWaypoint.fetch = collector.getCollectorState() == collectorState;

            } else if (currentWaypoint.shoot) {

                if (autoDrive.getDistance(0, 0) > autoDrive.inchesToMeters(240)) {
                    double angularVelocity = autoDrive
                            .turnTo(autoDrive.getHeading(currentWaypoint.x, currentWaypoint.y), drivetrain.getYaw());
                    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.2, 0.0, angularVelocity,
                            drivetrain.getGyroscopeRotation()));

                } else if (autoDrive.getDistance(0, 0) < autoDrive.inchesToMeters(84)) {
                    double angularVelocity = autoDrive
                            .turnTo(autoDrive.getHeading(currentWaypoint.x, currentWaypoint.y), drivetrain.getYaw());
                    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-0.2, 0.0, angularVelocity,
                            drivetrain.getGyroscopeRotation()));

                } else {
                    double angularVelocity = autoDrive
                            .turnTo(autoDrive.getHeading(currentWaypoint.x, currentWaypoint.y), drivetrain.getYaw());
                    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, angularVelocity,
                            drivetrain.getGyroscopeRotation()));

                    if (collectorState == CollectorState.NO_BALLS_LOADED) {
                        currentWaypoint.done = true;
                    }
                }

            } else {
                currentWaypoint.done = true;
            }

        } else if (!waypoints.isEmpty()) {
            currentWaypoint = waypoints.get(0);

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
