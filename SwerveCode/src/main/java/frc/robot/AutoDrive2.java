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

    private ArrayList<Pose2d> waypoints;
    /**
     * 
     * @param kPX
     * @param kDX
     * @param kIX
     * @param kPY
     * @param kDY
     * @param kIY
     * @param maxV
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
        this.wayPoints = new ArrayList<Pose2d>();
    }

    /**
     * 
     * @param goal
     * @param robot
     * @return
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

        return new ChassisSpeeds(velocityX, velocityY, 0);
    }

    public double getDistance(Pose2d a, Pose2d b) {
        return Math.sqrt(Math.pow(a.getX() - b.getX(), 2) + Math.pow(a.getY() - b.getY(), 2));
    }
    public ChassisSpeeds moveToWayPoint(Pose2d robot) {
        if(!this.waypoints.isEmpty()){
            currentGoal = waypoints.remove(0);
        }
        return this.moveTo(this.currentGoal, robot)
    }
    public void addWaypoint(Pose2d point){
        this.waypoints.add(point);
    }
}