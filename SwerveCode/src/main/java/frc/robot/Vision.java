package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController; //this puts in the xbox contoller stuff
import edu.wpi.first.wpilibj.GenericHID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Vision {
  //constantse
  private static double TURRET_ERROR = 0.5;           // Allowed aiming error in degrees
  private static double LOCK_ERROR = 1.0;
  private static double TURRET_ROTATE_KP = 18.0;   // Proportional constant for turret rotate speed
  private static double RPM_TO_TICKS_MS = 2048.0/600.0;  // Conversion factor for rotational velocity
  private static double TRIGGER_MOTOR_SPEED = 0.4;       // Maximum power for the motor feeding the flywheel
  private static double SHOOTING_RPM_RANGE = 20;         // Allowed RPM error for flywheel
  //
  private static double CAMERA_HEIGHT = 16.0;            // Limelight height above ground (inches)
  private static double CAMERA_ANGLE  = 25.0;            // Limelight camera angle above horizontal (degrees)
  private static double TARGET_HEIGHT = 98.25;           // Center of target above ground (inches)
  private static double TARGET_HEIGHT_DELTA = TARGET_HEIGHT - CAMERA_HEIGHT;
  //
  private static double MANUAL_POWER = 0.5;             // Turret power for manual control
  //
  private static double TURRET_TICKS_PER_DEGREE = 1770 / 90;  // Encoder ticks per degree

  // Motor controllers
  public WPI_TalonSRX turretRotate;                     // Motor for rotating the turret
  // Network Table entries
  private NetworkTableEntry tx;                         // Angle error (x) from LimeLight camera
  private NetworkTableEntry ty;                         // Angle error (y) from LimeLight camera
  private NetworkTableEntry ta;                         // Target area measurement from LimeLight camera
  private NetworkTableEntry tv;                         // Target valid indicator from Limelight camera
  // Shared variables
  public boolean targetValid;     // Indicate when the Limelight camera has found a target
  public boolean targetLocked;    // Indicate when the turret is centered on the target
  public double  targetRange;     // Range from robot to target (inches)
  //Private autoaim variables
   private double  turretSpeed;
   private double  xError;
   private double  yError;
   private double  area;

  public static boolean autonomousEnabled;
  
  /**
   * This constructor will intialize the motors and internal variables for the robot turret
   */
  public Vision() {

    //set up networktables for limelight
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    // Establish initial values for variables we share
    targetValid   = false;
    targetLocked  = false;
    targetRange   = 0.0;
  }


  /**
   * Converts the RPM input to the ticks/100ms velocity metric used by the
   * Falcon 500 motor
   * 
   * @param rpm Rotational velocity in Revolutions Per Minute (RPM)
   * @return Encoder ticks per 100msec
   */
  public double rpmToFalcon(double rpm){
    return rpm * RPM_TO_TICKS_MS;
  }


  /**
   * Converts the internal Falcon 500 velocity measurement of ticks per 100msec
   * into Revolutions Per Minute (RPM)
   * @param falcon Rotational velocity in ticks/100msec.  This is usually read from the controller
   * @return Rotational velocity in RPM
   */
  public double falconToRPM(double falcon){
    return falcon / RPM_TO_TICKS_MS;
  }

  /**
   * Read the target x error from the Limelight camera and move the turret until the error is 0
   * 
   * @param ballShooterController Used to enable autoAim turret motion
   */
  public double autoAim() {

    // Read the Limelight data from the Network Tables
    xError      = tx.getDouble(0.0);
    yError      = ty.getDouble(0.0);
    area        = ta.getDouble(0.0);
    targetValid = (tv.getDouble(0.0) != 0); // Convert the double output to boolean
    // Compute range to target.
    // Formula taken from https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
    targetRange = TARGET_HEIGHT_DELTA / Math.tan(Math.toRadians(CAMERA_ANGLE + yError));
    
    // Setting power based on the xError causes the turret to slow down as the error approaches 0
    // This prevents the turret from overshooting 0 and oscillating back and forth
    // KP is a scaling factor that we tested
    turretSpeed = Math.toRadians(xError) * TURRET_ROTATE_KP;

    turretSpeed = Math.max(turretSpeed, -2);
    turretSpeed = Math.min(turretSpeed, 2);

    if (Math.abs(xError) < LOCK_ERROR) {               // Turret is pointing at target (or no target)
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
    SmartDashboard.putNumber("Turret Speed", turretSpeed);

    //x = 0 when the camera sees the target is in the center
    // Only allow the turret to track when commanded
    if (Math.abs(xError) < TURRET_ERROR) {               // Turret is pointing at target (or no target)
      return 0.0; //Returns 0 if the turret is within the margin error
    }
    else {
      return turretSpeed;
    }

    
  }

  
}