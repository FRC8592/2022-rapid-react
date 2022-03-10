package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;

public class Vision {
  
  //constants passed in during initilization 
  private double lockError;  
  private double cameraHeight;
  private double cameraAngle;
  private double targetHeight;
  private double rotationKP;
  // Network Table entries
  private NetworkTableEntry tx;   // Angle error (x) from LimeLight camera
  private NetworkTableEntry ty;   // Angle error (y) from LimeLight camera
  private NetworkTableEntry ta;   // Target area measurement from LimeLight camera
  private NetworkTableEntry tv;   // Target valid indicator from Limelight camera
  // Shared variables
  public boolean targetValid;     // Indicate when the Limelight camera has found a target
  public boolean targetLocked;    // Indicate when the turret is centered on the target
  public double  targetRange;     // Range from robot to target (inches)
  public Timer timer;
  private double processedDx = 0;
  private double processedDy = 0;
  //Private autoaim variables
   private double turnSpeed;
   private double lastTime  = 0;
   private double xtime     = 0;
   private double lastAngle = 0;
   private double changeInAngleError = 0;

   //constants for averaging limelight averages
   private int MIN_LOCKS = 3;
   private int STAT_SIZE = 5; 

   private LinkedList<LimelightData> previousCoordinates;

   private String limelightName;

  // Pipeline constants
   private static int BLUE_PIPELINE = 1;
   private static int RED_PIPELINE = 0;

   private final double DEG_TO_RAD = 0.0174533;
   private final double IN_TO_METERS = 0.0254;
  

  /**
   * This constructor will intialize internal variables for the robot turret
   */
  public Vision(String limelightName, double lockError, double cameraHeight,
                double cameraAngle, double targetHeight, double rotationKP) {

    // Set up networktables for limelight
    NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    previousCoordinates = new LinkedList<LimelightData>();

    // Establish initial values for variables we share
    targetValid   = false;
    targetLocked  = false;
    targetRange   = 0.0;
    timer = new Timer();
    timer.start();

    this.limelightName = limelightName;
    this.lockError     = lockError;
    this.cameraHeight  = cameraHeight;
    this.cameraAngle   = cameraAngle;
    this.targetHeight  = targetHeight;
    this.rotationKP    = rotationKP;
  }


  public void updateVision(){      //method should be called continuously during autonomous and teleop
    double xError;
    double yError;
    double area;
    double totalDx = 0;
    double totalDy = 0;
    int totalValid = 0;

    // Read the Limelight data from the Network Tables
    xError      = tx.getDouble(0.0);
    yError      = ty.getDouble(0.0);
    area        = ta.getDouble(0.0);
    targetValid = (tv.getDouble(0.0) != 0); // Convert the double output to boolean

    //generates average of limelight parameters
    previousCoordinates.add(new LimelightData(xError, yError, targetValid));
    if (previousCoordinates.size() > STAT_SIZE){
      previousCoordinates.removeFirst();
    }

    for(LimelightData data: previousCoordinates){
      if (data.ballValid == true){
        totalDx = data.dx + totalDx;
        totalDy = data.dy + totalDy;
        totalValid = totalValid +1;
      }
    }

    processedDx = totalDx/totalValid;
    processedDy = totalDy/totalValid;
    targetValid = (totalValid >= MIN_LOCKS);

    targetRange = distanceToTarget();

    if (Math.abs(processedDx) < lockError){          // Turret is pointing at target (or no target)
      targetLocked = targetValid;               // We are only locked when targetValid
    }           
    else{
      targetLocked = false;
    }

    //post driver data to smart dashboard periodically
    SmartDashboard.putNumber(limelightName + "/xerror in radians", Math.toRadians(xError));
    SmartDashboard.putNumber(limelightName + "/LimelightX", xError);
    SmartDashboard.putNumber(limelightName + "/LimelightY", yError);
    SmartDashboard.putNumber(limelightName + "/LimelightArea", area);
    SmartDashboard.putBoolean(limelightName + "/Target Valid", targetValid);
    SmartDashboard.putNumber(limelightName + "/Change in Angle Error", changeInAngleError);
    SmartDashboard.putNumber(limelightName + "/Average Y", processedDy);
    SmartDashboard.putNumber(limelightName + "/Average X", processedDx);
    SmartDashboard.putNumber(limelightName + "/Total Valid", totalValid);
    SmartDashboard.putNumber(limelightName + "/Target Range", targetRange);
    SmartDashboard.putBoolean(limelightName + "/Target Locked", targetLocked);
  }


  public double delta(){ //gets the change in angle over time(seconds)
    xtime = timer.get(); 
    changeInAngleError = (processedDx - lastAngle)/(xtime - lastTime);
    lastAngle = processedDx;  // reset initial angle
    lastTime  = xtime;        // reset initial time
    return changeInAngleError;
  }


  /**
   * Gives distance from the robot to the target in meters
   * Compute range to target.
   * Formula taken from https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   * @return distance in meters
   */
  public double distanceToTarget(){
    if (targetValid){
      double distanceInches = (targetHeight - cameraHeight) / Math.tan((cameraAngle + processedDy) * DEG_TO_RAD);//Equation is from limelight documentation finding distance
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
    return Math.toRadians(processedDx);
  }

  public double turnRobot(){
    if (targetValid){
      // Setting power based on the xError causes the turret to slow down as the error approaches 0
      // This prevents the turret from overshooting 0 and oscillating back and forth
      // KP is a scaling factor that we tested
      turnSpeed = Math.toRadians(processedDx) * rotationKP; // + turret_rotate_kd*delta();
      turnSpeed = Math.max(turnSpeed, -4);
      turnSpeed = Math.min(turnSpeed, 4);

      //
      // Set a minimum turnSpeed so that we don't get stuck when close to zero error
      //
      if (turnSpeed > 0){
        turnSpeed = Math.max(turnSpeed, Constants.MIN_TURN_SPEED);
      }
      else{
        turnSpeed = Math.min(turnSpeed, -Constants.MIN_TURN_SPEED);
      }
    }
    else{
      turnSpeed = 1;    // Spin in a circle until a target is located
    }
    
    //
    // Stop turning immediately when target is locked (at center)
    //
    if (targetLocked){
      turnSpeed = 0;
    }
    SmartDashboard.putNumber(limelightName + "/Turret Speed", turnSpeed);

    return -turnSpeed;
  }

  //
  // Drive towards the target
  // This should probably be updated to base speed on distance from the target
  //
  public double moveTowardsTarget(){
    double moveSpeed = 0.0;
  
    if (targetLocked == true){
      moveSpeed = -.5;
    }
    SmartDashboard.putNumber(limelightName + "/Move Speed", moveSpeed);
    return moveSpeed;
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