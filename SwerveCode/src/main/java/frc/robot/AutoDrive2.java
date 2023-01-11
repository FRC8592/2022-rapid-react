package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.lang.Math;

public class AutoDrive2 {
    // Values for clamping and finishing actions
    private double maxVelocity; 
    private double maxTheta;
    private double acceptanceRadius;
    private boolean nextWaypoint;
    
    /*
    // PID Values
    private double kVelocityPX;
    private double kVelocityIX;
    private double kVelocityDX; 
    
    private double kVelocityPY;
    private double kVelocityIY;
    private double kVelocityDY;

    private double kOmegaP;
    private double kOmegaI;
    private double kOmegaD;
    */

    // PID Controllers
    private PIDController pidVelocityControlX;
    private PIDController pidVelocityControlY;
    private PIDController pidOmegaControl;

    // Goal to move towards
    private Pose2d currentGoal;

    // Smoothing filter to smooth velocity values
    private SmoothingFilter sf = new SmoothingFilter(5, 5, 5);

    private ArrayList<Pose2d> waypoints;
    /**
     * Construct object and assign values, PID values will default to 0, use pid set methods to change
     * 
     * @param maxV Max velocity the robot will move
     * @param maxOmega Max omegas (in radians) the robot will move
     * @param acceptanceRadius Largest distance before robot is considered at a waypoint
     */
    public AutoDrive2(double maxV, double maxTheta, double acceptanceRadius){
        /*
        kVelocityPX = 0;
        kVelocityIX = 0;
        kVelocityDX = 0;
        kVelocityPY = 0;
        kVelocityIY = 0;
        kVelocityDY = 0;
        kOmegaP = 0;
        kOmegaI = 0;
        kOmegaD = 0;  
        */
        maxVelocity = maxV;
        this.maxTheta = maxTheta;
        pidVelocityControlX = new PIDController(0, 0, 0);
        pidVelocityControlY = new PIDController(0, 0, 0);
        pidOmegaControl = new PIDController(0, 0, 0);
        this.acceptanceRadius = acceptanceRadius;
        this.waypoints = new ArrayList<Pose2d>();
    }

    /**
     * Return speed needed to move to a certain position
     * 
     * @param goal Position to move to
     * @param robot Current position
     * @return Velocities for each direction
     */
    public ChassisSpeeds moveTo(Pose2d goal, Pose2d robot) {
        double velocityX = 0;
        double velocityY = 0;
        double omega = 0;
        if(getDistance(goal, robot) >= this.acceptanceRadius) {
          velocityX = pidVelocityControlX.calculate(robot.getX(), goal.getX());
          velocityY = pidVelocityControlY.calculate(robot.getY(), goal.getY());
        
          velocityX = Math.max(Math.min(velocityX, maxVelocity), -maxVelocity);
          velocityY = Math.max(Math.min(velocityY, maxVelocity), -maxVelocity);
        }
        if(Math.abs(robot.getRotation().getRadians() - goal.getRotation().getRadians()) < 0.04) {
            omega = pidOmegaControl.calculate(robot.getRotation().getRadians(), goal.getRotation().getRadians());
            omega = Math.max(Math.min(omega, maxTheta), -maxTheta);
        }
        ChassisSpeeds cs = new ChassisSpeeds(velocityX, velocityY, omega);
        return sf.smooth(cs);
    }

    /**
     * Distance formula
     * 
     * @param a Point A
     * @param b Point B
     * @return Distance between A and B
     */
    public double getDistance(Pose2d a, Pose2d b) {
        return Math.sqrt(Math.pow(a.getX() - b.getX(), 2) + Math.pow(a.getY() - b.getY(), 2));
    }

    /**
     * Move to the next waypoint using moveTo method
     * 
     * @param robot Current position of robot
     * @return Velocities for each direction
     */
    public ChassisSpeeds moveToWayPoint(Pose2d robot) {
        if( nextWaypoint && !this.waypoints.isEmpty()){
            nextWaypoint = false;
            currentGoal = waypoints.remove(0);
        }
        nextWaypoint = (this.getDistance(robot, currentGoal) < this.acceptanceRadius);
        return this.moveTo(this.currentGoal, robot);
    }

    /**
     * Add a waypoint to the movement path
     * 
     * @param point Point to move to
     */
    public void addWaypoint(Pose2d point){
        this.waypoints.add(point);
    }
    public void initWaypoints(){
        this.waypoints = new ArrayList<Pose2d>();
        this.nextWaypoint = true;
    }

    /**
     * Set the PID values in the X direction 
     * 
     * @param p 
     * @param i 
     * @param d 
     */
    public void setXPIDValues(double p, double i, double d){
        pidVelocityControlX.setP(p);
        pidVelocityControlX.setI(i);
        pidVelocityControlX.setD(d);
    }
    
    /**
     * Set the PID values in the Y direction 
     * 
     * @param p 
     * @param i 
     * @param d 
     */
    public void setYPIDValues(double p, double i, double d){
        pidVelocityControlY.setP(p);
        pidVelocityControlY.setI(i);
        pidVelocityControlY.setD(d);
    }

    /**
     * Set the PID values for rotation
     * 
     * @param p 
     * @param i 
     * @param d 
     */
    public void setRotationPIDValues(double p, double i, double d){
        pidOmegaControl.setP(p);
        pidOmegaControl.setI(i);
        pidOmegaControl.setD(d);
    }
    
}