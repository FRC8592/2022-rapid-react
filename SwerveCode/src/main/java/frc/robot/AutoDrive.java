package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
/**
 * @author gavin malzahn
 * @author audrey chiang
 * FRC Season 2022
 */
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;

public class AutoDrive {

    double robotRotation;                        //angle of robot relative to global field  
    private static double hubCenterX;            //xcenter of the hub in field coordinates
    private static double hubCenterY;            //ycenter of the hub in field coordinates
    private static double hubRadius = .609;
    private boolean isGoodData;                  //can this data be used
    private double positionX;                    //position in x;
    private double positionY;                    //position in y;
    private final double KP_velocity_X = 5;
    private final double KP_velocity_Y = 5;
    private Drivetrain driveTrain;
    private final double KD_velocity_X = 0;
    private final double KD_velocity_Y = 0;
    private final double KP_angular_velocity = 0.5;
    private final double KD_angular_velocity = 0;
    private double lastErrorX;
    private double lastErrorY;
    private double lastAngleError;
    private double targetX;
    private double targetY;
    public Timer moveTimer;
    public Timer turnTimer;
    public double lastMoveTime;
    public double lastTurnTime;
    private Drivetrain drive;
    private PIDController xVelocityPID;
    private PIDController yVelocityPID;
    private PIDController angularVelocityPID;


    /**
     * 
     * @param hubCenterX
     * @param hubCenterY
     */
    public AutoDrive(double hubCenterX, double hubCenterY, Drivetrain drive) { 
        AutoDrive.hubCenterX = hubCenterX;
        AutoDrive.hubCenterY = hubCenterY;
        this.robotRotation = 0;
        AutoDrive.hubRadius = .6;
        isGoodData = false;
        this.driveTrain = drive;
        targetX = 0;
        targetY = 0;
        lastErrorY = 0;
        lastErrorX = 0;
        moveTimer = new Timer();
        turnTimer = new Timer();
        moveTimer.start();
        turnTimer.start();
        lastMoveTime = moveTimer.get();
        lastTurnTime = turnTimer.get();
        this.drive = drive;
        //PID Control
        this.xVelocityPID = new PIDController(Constants.MOVE_TO_VELOCITY_P,Constants.MOVE_TO_VELOCITY_I, Constants.MOVE_TO_VELOCITY_D);
        this.yVelocityPID = new PIDController(Constants.MOVE_TO_VELOCITY_P,Constants.MOVE_TO_VELOCITY_I, Constants.MOVE_TO_VELOCITY_D);
        this.angularVelocityPID = new PIDController(Constants.MOVE_TO_VELOCITY_P,Constants.MOVE_TO_VELOCITY_I, Constants.MOVE_TO_VELOCITY_D);
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
        targetDistance = this.inchesToMeters(targetDistance);
        boolean testNone = false;
        if(vision.targetLocked){
            double distance2 = ((targetDistance)/Math.cos(targetOffsetRotation)) + (AutoDrive.hubRadius - .177);
            this.positionX = -distance2 * Math.cos(robotRotationRad + targetOffsetRotation) + hubCenterX;
            this.positionY = distance2 * Math.sin(robotRotationRad + targetOffsetRotation) + hubCenterY;
            //this.isGoodData = true;
    /*
            double xCam = targetDistance;
            double yCam = xCam/(Math.tan(targetOffsetRotation));
            double hCam = Math.sqrt(Math.pow(xCam,2.0) + Math.pow(yCam,2.0)) + AutoDrive.hubRadius;
            this.positionX = -hCam*Math.cos(targetOffsetRotation + robotRotationRad);
            this.positionY = -hCam*Math.sin(targetOffsetRotation + robotRotationRad);
            */
            Pose2d pose = new Pose2d(this.positionX, this.positionY, new Rotation2d());
        
            drive.resetPose(pose);
            this.isGoodData = true;
            
        } else {
            this.isGoodData = true;
            Pose2d pose = drive.getCurrentPos();
            this.positionX = pose.getX();
            this.positionY = pose.getY();
        }
        SmartDashboard.putNumber("Yaw value", robotRotation);
        SmartDashboard.putNumber("Position Y", this.metersToInches(positionY));
        SmartDashboard.putNumber("Position X", this.metersToInches(positionX));
       // SmartDashboard.putNumber("Position Valid", isGoodData ? 1.0: 0.0);
        
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
    /**
     * sets angular velocity to a given angle in Yaw using PID control
     * @param targetAngle
     * @param currentAngle
     * @return
     */
    public double turnTo(double targetAngle, double currentAngle){
        double target = targetAngle;
        double angleError = targetAngle - currentAngle;
       //Need to change Angle SP to find shortest path
       //Currently in Degrees
        if(angleError > 180){
           target -= 360;
        } else if(angleError < -180){
            target += 360;
        }
        angleError = target - currentAngle;
      
        double angleVelocity = angularVelocityPID.calculate(currentAngle, target);
        angleVelocity = Math.max(angleVelocity, -ConfigRun.MAX_TURN_SPEED);
        angleVelocity = Math.min(angleVelocity, ConfigRun.MAX_TURN_SPEED);
        if(Math.abs(angleError) < 5){
            angleVelocity = 0;
        }
        return angleVelocity;
    }
    
    /**
     * sets Velocity values using PID control based on setPoint
     * @param setPointX
     * @param setPointY
     * @return
     */
    public double[] moveTo(double setPointX, double setPointY){
        double[] velocity = new double [2]; 
        velocity[0] = 0;
        velocity[1] = 0;   
        //updatePosition(robotRotation, vision);
        if(isGoodData){

            velocity[0] = xVelocityPID.calculate(positionX, setPointX);
            velocity[0] = Math.max(velocity[0], -ConfigRun.MAX_MOVE_SPEED);
            velocity[0] = Math.min(velocity[0], ConfigRun.MAX_MOVE_SPEED);

            velocity[1] = xVelocityPID.calculate(positionY, setPointY);
            velocity[1] = Math.max(velocity[1], -ConfigRun.MAX_MOVE_SPEED);
            velocity[1] = Math.min(velocity[1], ConfigRun.MAX_MOVE_SPEED);
        }
        return velocity;
    }

    public double getDistance(double x, double y){
        return Math.sqrt(Math.pow((x - getX()),2) + Math.pow((y - getY()), 2));
    }

    public double getHeading(double x, double y){
        return Math.toDegrees(Math.atan2(y - getY(), x - getX()));
    }

    public double inverseHeading(double x, double y){
        return Math.toDegrees(Math.atan2(getY() - y, getX()- x));
    }

    public double inchesToMeters(double inch){
        return inch*0.0254;
    }

    public double metersToInches(double meters){
        return meters*39.3701;
    }
    
}
