package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
/**
 * @author gavin malzahn
 * @author audrey chiang
 * FRC Season 2022
 */
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Locality {

    double robotRotation;                        //angle of robot relative to global field  
    private static double hubCenterX;            //xcenter of the hub in field coordinates
    private static double hubCenterY;            //ycenter of the hub in field coordinates
    private static double hubRadius = .6;
    private boolean isGoodData;                  //can this data be used
    private double positionX;                    //position in x;
    private double positionY;                    //position in y;
    private final double KP_velocity_X = 0.5;
    private final double KP_velocity_Y = 0.5;
    private Drivetrain driveTrain;

    /**
     * 
     * @param hubCenterX
     * @param hubCenterY
     */
    public Locality(double hubCenterX, double hubCenterY, Drivetrain drive) { 
        Locality.hubCenterX = hubCenterX;
        Locality.hubCenterY = hubCenterY;
        this.robotRotation = 0;
        Locality.hubRadius = .6;
        isGoodData = false;
        this.driveTrain = drive;
    }

    // Needs to be called every update

    /***
     * 
     * @param robotRotation
     * @param vision
     * @param SmartDashboard 
     */
    
    public void updatePosition(double robotRotation, Vision vision){     
        double targetDistance = vision.distanceToTarget(); 
        double targetOffsetRotation = vision.offsetAngle(); 
        double robotRotationRad = Math.toRadians(robotRotation);
        if(vision.targetValid){
            double distance2 = (targetDistance + Locality.hubRadius)/Math.cos(targetOffsetRotation);
            this.positionX = -distance2 * Math.cos(robotRotationRad + targetOffsetRotation) + hubCenterX;
            this.positionY = -distance2 * Math.sin(robotRotationRad + targetOffsetRotation) + hubCenterY;
            this.isGoodData = true;
        } else {
            Pose2d pose = this.driveTrain.updatePose();
            this.positionX = pose.getX();
            this.positionY = pose.getY();
            this.isGoodData = false;
        }
        SmartDashboard.putNumber("Yaw value", robotRotation);
        SmartDashboard.putNumber("Position Y", positionY);
        SmartDashboard.putNumber("Position X", positionX);
        SmartDashboard.putNumber("Position Valid", isGoodData ? 1.0: 0.0);
        
    }

    /**
     * 
     * @return
     */
    public double getX(){
        return this.positionX;
    }

    /***
     * 
     * @return
     */
    public double getY(){
        return this.positionY;
    }

    /***
     * 
     * @return
     */
    public boolean isGood(){
        return this.isGoodData;
    }
    public double[] moveTo(double setPointX, double setPointY, Vision vision){
        double[] velocity = new double [2]; 
        velocity[0] = 0;
        velocity[1] = 0;
        
        updatePosition(robotRotation, vision);
        if(isGoodData){
            double errorX = setPointX - this.positionX;
            double errorY = setPointY - this.positionY;
            velocity[0] = errorX * KP_velocity_X;
            velocity[0] = Math.min(velocity[0], -1.7);
            velocity[0] = Math.max(velocity[0], 1.7);
            velocity[1] = errorY * KP_velocity_Y;
            velocity[1] = Math.min(velocity[1], -1.7);
            velocity[1] = Math.max(velocity[1], 1.7);
        }
        return velocity;
    }
    
}
