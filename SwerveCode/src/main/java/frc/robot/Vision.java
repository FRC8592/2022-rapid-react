package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.ALLIANCE_COLOR;

import java.util.LinkedList;

public class Vision {
  
  //constants passed in during initilization 
  private double lockError;  
  private double cameraHeight;
  private double cameraAngle;
  private double targetHeight;
  private double rotationKP;

  // Network Table entries
  private NetworkTableEntry tx;                         // Angle error (x) from LimeLight camera
  private NetworkTableEntry ty;                         // Angle error (y) from LimeLight camera
  private NetworkTableEntry ta;                         // Target area measurement from LimeLight camera
  private NetworkTableEntry tv;                         // Target valid indicator from Limelight camera
  // Shared variables
  public boolean targetValid;     // Indicate when the Limelight camera has found a target
  public boolean targetLocked;    // Indicate when the turret is centered on the target
  public double  targetRange;     // Range from robot to target (inches)
  public Timer timer;
  //Private autoaim variables
   private double turnSpeed;
   private double xError;
   private double yError;
   private double area;
   private double lastTime = 0;
   private double xtime = 0;
   private double lastAngle = 0;
   private double changeInAngleError = 0;

  private LinkedList<LimelightData> previousCoordinates;

  //pipeline constants
   private static int BLUE_PIPELINE = 1;
   private static int RED_PIPELINE = 0;

   private final double DEG_TO_RAD = 0.0174533;
  private final double IN_TO_METERS = 0.0254;

  public ColorSensor colorSensor;
  
  /**
   * This constructor will intialize internal variables for the robot turret
   */
  public Vision(String limelightName, double lockError, double cameraHeight, double cameraAngle, double targetHeight, double rotationKP) {

    //set up networktables for limelight
    NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    // Establish initial values for variables we share
    targetValid   = false;
    targetLocked  = false;
    targetRange   = 0.0;
    timer = new Timer();
    timer.start();

    this.lockError      = lockError;
    this.cameraHeight   = cameraHeight;
    this.cameraAngle    = cameraAngle;
    this.targetHeight   = targetHeight;
    this.rotationKP     = rotationKP;

  }


  public double delta(){ //gets the change in angle over time(seconds)
    xtime = timer.get(); 
    changeInAngleError = (xError - lastAngle)/(xtime - lastTime);
    lastAngle = xError; // reset initial angle
    lastTime = xtime; // reset initial time
    return changeInAngleError;

  }


  /**
   * Gives distance from the robot to the target in meters
   * Compute range to target.
   * Formula taken from https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   * @return distance in meters
   */
  public double distanceToTarget(double averageDy){
    //targetValid = (tv.getDouble(0.0) != 0); // Convert the double output to boolean
    //double targetAngle = targetAngle.getDouble(0.0);
    if (targetValid){
      double distanceInches = (targetHeight - cameraHeight) / Math.tan((cameraAngle + averageDy) * DEG_TO_RAD);//Equation is from limelight documentation finding distance
      return distanceInches * IN_TO_METERS;
    }
    return -1;
  }


  public double distanceToTarget(){
    targetValid = (tv.getDouble(0.0) != 0); // Convert the double output to boolean
    double targetAngle = ty.getDouble(0.0);
    if (targetValid){
      double distanceInches = (targetHeight - cameraHeight) / Math.tan((cameraAngle + targetAngle) * DEG_TO_RAD);//Equation is from limelight documentation finding distance
      return distanceInches * IN_TO_METERS;
    }
    return -1;
  }


  /**
   * 
   * 
   * @return angle offset in radians 
   */
  public double offsetAngle(){
    double offsetAngle = tx.getDouble(0.0);

    return Math.toRadians(offsetAngle);
  }


  public double autoAim() {

    // Read the Limelight data from the Network Tables
    xError      = tx.getDouble(0.0);
    yError      = ty.getDouble(0.0);
    area        = ta.getDouble(0.0);
    targetValid = (tv.getDouble(0.0) != 0); // Convert the double output to boolean

    double totalDx = 0.0;
    double totalDy = 0.0;
    int totalBallValid = 0;

    previousCoordinates.add(new LimelightData(xError, yError, targetValid));
    if (previousCoordinates.size() > 5){
      previousCoordinates.removeFirst();
    }

    for(LimelightData data: previousCoordinates){
      if (data.ballValid == true){
        totalDx = data.dx + totalDx;
        totalDy = data.dy + totalDy;
        totalBallValid = totalBallValid +1;
      }
    }

    double averageDx = totalDx/totalBallValid;
    double averageDy = totalDy/totalBallValid;

    if (targetValid){
      targetRange = distanceToTarget(averageDy);
    
      // Setting power based on the xError causes the turret to slow down as the error approaches 0
      // This prevents the turret from overshooting 0 and oscillating back and forth
      // KP is a scaling factor that we tested
      turnSpeed = Math.toRadians(averageDx) * rotationKP; // + turret_rotate_kd*delta();
      turnSpeed = Math.max(turnSpeed, -4);
      turnSpeed = Math.min(turnSpeed, 4);
    }
    else if(lastAngle != 0){
      turnSpeed = Math.toRadians(lastAngle) * rotationKP;
      turnSpeed = Math.max(turnSpeed, -4);
      turnSpeed = Math.min(turnSpeed, 4);
    }
    else{
      turnSpeed = 1;
    }
    
    if (Math.abs(xError) < lockError) {               // Turret is pointing at target (or no target)
      targetLocked = targetValid;                     // We are only locked when targetValid
    }
    else{
      targetLocked = false;
    }

    //post driver data to smart dashboard periodically
    SmartDashboard.putNumber("xerror in radians", Math.toRadians(xError));
    SmartDashboard.putNumber("LimelightX", xError);
    SmartDashboard.putNumber("LimelightY", yError);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Target Range", targetRange);
    SmartDashboard.putBoolean("Target Valid", targetValid);
    SmartDashboard.putBoolean("Target Locked", targetLocked);
    SmartDashboard.putNumber("Turret Speed", turnSpeed);
    SmartDashboard.putNumber("Change in Angle Error", changeInAngleError);
    SmartDashboard.putNumber("Average Y", averageDy);
    SmartDashboard.putNumber("Average X", averageDx);
    SmartDashboard.putNumber("Total Valid", totalBallValid);


    return turnSpeed;
  }

  private class LimelightData{ 
    double dx;
    double dy;
    boolean ballValid;

    public LimelightData(double xError, double yError, boolean ballValid){
      dx = xError;
      dy = yError;
      this.ballValid = ballValid;
    }

  }
}