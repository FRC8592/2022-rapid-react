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
    public boolean firstLock;

    public Waypoint(double x, double y, double acceptRadius, boolean turnTo,boolean fetch, boolean shoot){
        this.x = x;
        this.y = y;
        this.acceptRadius = acceptRadius;
        this.turnTo = turnTo;
        this.shoot = shoot;
        this.here = false;
        this.done = false;
        this.fetch = fetch;
        firstLock = false;
    }
    public Waypoint(boolean firstLock){
        this.firstLock = true;
        this.x = 0;
        this.y = 0;
        this.acceptRadius = 0;
        this.turnTo = false;
        this.shoot = false;
        this.here = false;
        this.done = false;
        this.fetch = false;

    }
}
