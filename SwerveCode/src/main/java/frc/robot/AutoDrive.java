package frc.robot;

import java.util.ArrayList;

import frc.robot.Collector.CollectorState;

public class AutoDrive {
    private ArrayList<Waypoint> waypoints;
    private Waypoint currentWaypoint;
    private Locality locality;
    private Drivetrain drivetrain;
    private Collector collector;
    private Shooter shooter;
    private Vision vision;
    private CollectorState collectorState;

    public AutoDrive(Locality locality, Drivetrain drivetrain, Collector collector, Shooter shooter, Vision vision){
        waypoints = new ArrayList<Waypoint>();
    }

    public void addWaypoint(Waypoint waypoint){
        waypoints.add(waypoint);
    }

    public void runWaypoint(){
        if(currentWaypoint != null && !currentWaypoint.done){
            if(!currentWaypoint.here){
            double distance = locality.getDistance(currentWaypoint.x, currentWaypoint.y);
            currentWaypoint.here = distance <= currentWaypoint.acceptRadius;
            collectorState = collector.determineCollectorState();
            }else if(currentWaypoint.fetch){
                vision.moveTowardsTarget();
                currentWaypoint.fetch = collector.determineCollectorState() == collectorState;
            }else if(currentWaypoint.shoot){
                if(locality.getDistance(0, 0) > 20){
                    locality.getHeading(currentWaypoint.x, currentWaypoint.y);
                    locality.
                    drivetrain.drive();
                }else if(locality.getDistance(0, 0) < 7){

                }
            }


        }
        else if(!waypoints.isEmpty()){
                currentWaypoint = waypoints.get(0);
                if(currentWaypoint.turnTo){
                   currentWaypoint.heading = locality.getHeading(currentWaypoint.x, currentWaypoint.y);
                collectorState = collector.determineCollectorState();
                }
                waypoints.remove(0);
            }else{
                return;
            }
        }
        

    }
}
