package frc.robot;
/**
 * @author gavin malzahn
 * @author audrey chiang
 * FRC Season 2022
 */
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

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
    private final double KD_velocity_X = 0;
    private final double KD_velocity_Y = 0;
    private double lastErrorX;
    private double lastErrorY;
    private double targetX;
    private double targetY;
    public Timer timer;
    public double lastTime;


    /**
     * 
     * @param hubCenterX
     * @param hubCenterY
     */
    public Locality(double hubCenterX, double hubCenterY) { 
        Locality.hubCenterX = hubCenterX;
        Locality.hubCenterY = hubCenterY;
        this.robotRotation = 0;
        Locality.hubRadius = .6;
        isGoodData = false;
        targetX = 0;
        targetY = 0;
        lastErrorY = 0;
        lastErrorX = 0;
        timer = new Timer();
        timer.start();
        lastTime = timer.get();
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
    
        if(targetX != setPointX || targetY != setPointY){
            lastErrorX = 0;
            lastErrorY = 0;

        }


        double[] velocity = new double [2]; 
        velocity[0] = 0;
        velocity[1] = 0;
        
        updatePosition(robotRotation, vision);
        if(isGoodData){
            double errorX = setPointX - this.positionX;
            double errorY = setPointY - this.positionY;

            double xtime = timer.get(); 
            double changeInErrorX = (errorX - lastErrorX)/(xtime - lastTime);
            lastErrorX = errorX;  // reset initial angle

            double changeInErrorY = (errorY - lastErrorY)/(xtime - lastTime);
            lastErrorY = errorY;  // reset initial angle

            velocity[0] = errorX * KP_velocity_X + KD_velocity_X*lastErrorX;
            velocity[0] = Math.min(velocity[0], -1.7);
            velocity[0] = Math.max(velocity[0], 1.7);
            velocity[1] = errorY * KP_velocity_Y + KD_velocity_Y*lastErrorY;
            velocity[1] = Math.min(velocity[1], -1.7);
            velocity[1] = Math.max(velocity[1], 1.7);
            lastTime  = xtime;        // reset initial time

        }

        return velocity;
    }
    
}
