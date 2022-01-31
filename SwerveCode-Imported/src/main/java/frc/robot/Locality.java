package frc.robot;
/**
 * @author gavin malzahn
 * @author audrey chiang
 * FRC Season 2022
 */

public class Locality {

    double robotRotation;                        //angle of robot relative to global field  
    private static double hubCenterX;            //xcenter of the hub in field coordinates
    private static double hubCenterY;            //ycenter of the hub in field coordinates
    private static double hubRadius = .6;
    private boolean isGoodData;                  //can this data be used
    private double positionX;                    //position in x;
    private double positionY;                    //position in y;

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
    }

    // Needs to be called every update

    /***
     * 
     * @param robotRotation
     * @param vision
     */
    public void updatePosition(double robotRotation, Vision vision){     
        double targetDistance = 1; // TODO = vision.getDistance()
        double targetOffsetRotation = 1; //TODO = vision.xAngle()\
        if(vision.targetLocked){
        double distance2 = (targetDistance + Locality.hubRadius)/Math.cos(targetOffsetRotation);
        this.positionX = -distance2 * Math.cos(robotRotation + targetOffsetRotation) + hubCenterX;
        this.positionY = -distance2 * Math.sin(robotRotation + targetOffsetRotation) + hubCenterY;
        this.isGoodData = true;
        } else {
            this.isGoodData = false;
        }
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

    
}
