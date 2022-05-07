package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class Waypoint {
    public double x;
    public double y;
    public double acceptRadius;
    public double heading;
    public boolean turnTo;
    public boolean fetch;
    public boolean shoot;
    public boolean here;
    public boolean done;
    public boolean init;
    public Timer waypointTimer;
    public boolean lock;

    public Waypoint(double x, double y, double acceptRadius, boolean turnTo, boolean fetch, boolean shoot, boolean lock, Timer waypointTimer){
        this.x = x;
        this.y = y;
        this.acceptRadius = acceptRadius;
        this.turnTo = turnTo;
        this.shoot = shoot;
        this.here = false;
        this.done = false;
        this.waypointTimer = waypointTimer;
        this.fetch = fetch;
        this.lock = lock;
    }
}
