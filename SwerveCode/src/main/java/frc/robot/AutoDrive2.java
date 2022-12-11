package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;

public class AutoDrive2 {
    double maxVelocity; 
    double acceptanceRadius;
    
    double velocityKpX;
    double velocityKdX;
    double velocityKiX; 
    
    double velocityKpY;
    double velocityKdY;
    double velocityKiY;

    Pose2d currentGoal;

    PIDController pidVelocityControlX;
    PIDController pidVelocityControlY;
    SmoothingFilter sf = new SmoothingFilter(5);

    private ArrayList<Pose2d> waypoints;
    /**
     * Construct object and assign values
     * 
     * @param kPX P for X direction
     * @param kDX D for X direction
     * @param kIX I for X direction
     * @param kPY P for Y direction
     * @param kDY D for Y direction
     * @param kIY I for Y direction
     * @param maxV Max velocity the robot will move
     */
    public AutoDrive2(double kPX, double kDX, double kIX, 
                      double kPY, double kDY, double kIY, double maxV, double acceptanceRadius){
        velocityKpX = kPX;
        velocityKdX = kDX;
        velocityKiX = kIX;
        velocityKpY = kPY;
        velocityKdY = kDY;
        velocityKiY = kIY;
        maxVelocity = maxV;
        pidVelocityControlX = new PIDController(kPX, kIX, kDX);
        pidVelocityControlY = new PIDController(kPY, kIY, kDY);
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
        if(getDistance(goal, robot) >= this.acceptanceRadius) {
          velocityX = pidVelocityControlX.calculate(robot.getX(), goal.getX());
          velocityY = pidVelocityControlY.calculate(robot.getY(), goal.getY());
          velocityX = Math.max(Math.min(velocityX, maxVelocity), -maxVelocity);
          velocityY = Math.max(Math.min(velocityY, maxVelocity), -maxVelocity);
        }

        //return new ChassisSpeeds(velocityX, velocityY, 0);
        return sf.smooth(new ChassisSpeeds(velocityX, velocityY, 0));
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
        if(!this.waypoints.isEmpty()){
            currentGoal = waypoints.remove(0);
        }
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
}