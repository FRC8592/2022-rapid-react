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

    public AutoWaypoint(AutoDrive locality, Drivetrain drivetrain, Collector collector, Shooter shooter, Vision vision){
        waypoints = new ArrayList<Waypoint>();
        this.drivetrain = drivetrain;
        this.collector = collector;
        this.shooter = shooter;
        this.vision = vision;
    }

    public void addWaypoint(Waypoint waypoint){
        waypoints.add(waypoint);
    }

    public void runWaypoint(){
        if(currentWaypoint != null && !currentWaypoint.done){
            if(!currentWaypoint.here){
            double distance = autoDrive.getDistance(currentWaypoint.x, currentWaypoint.y);
            currentWaypoint.here = distance <= currentWaypoint.acceptRadius;
            collectorState = collector.determineCollectorState();
            autoDrive.moveTo(currentWaypoint.x, currentWaypoint.y);

            }else if(currentWaypoint.fetch){
                vision.moveTowardsTarget();
                currentWaypoint.fetch = collector.determineCollectorState() == collectorState;

            }else if(currentWaypoint.shoot){

                if(autoDrive.getDistance(0, 0) > 20){
                    double angularVelocity = autoDrive.turnTo(autoDrive.getHeading(currentWaypoint.x, currentWaypoint.y), drivetrain.getYaw());
                    drivetrain.drive(new ChassisSpeeds(0.5, 0.0, angularVelocity));

                }else if(autoDrive.getDistance(0, 0) < 7){
                    double angularVelocity = autoDrive.turnTo(autoDrive.getHeading(currentWaypoint.x, currentWaypoint.y), drivetrain.getYaw());
                    drivetrain.drive(new ChassisSpeeds(-0.5, 0.0, angularVelocity));

                }else{
                    double angularVelocity = autoDrive.turnTo(autoDrive.getHeading(currentWaypoint.x, currentWaypoint.y), drivetrain.getYaw());
                    drivetrain.drive(new ChassisSpeeds(0,0,angularVelocity));

                    if(collectorState == CollectorState.NO_BALLS_LOADED){
                        currentWaypoint.done = true;
                    }
                }

            } else {
                currentWaypoint.done = true;
            }


        }
        else if(!waypoints.isEmpty()){
                currentWaypoint = waypoints.get(0);

                if(currentWaypoint.turnTo){
                   currentWaypoint.heading = autoDrive.getHeading(currentWaypoint.x, currentWaypoint.y);
                collectorState = collector.determineCollectorState();
                }
                waypoints.remove(0);

            }else{

                drivetrain.drive(new ChassisSpeeds(0,0,0));
                return;
            }
        }
    }
