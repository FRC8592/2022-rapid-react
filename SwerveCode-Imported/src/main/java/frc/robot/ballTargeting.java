package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ALLIANCE_COLOR;
import edu.wpi.first.wpilibj.XboxController; //this puts in the xbox contoller stuff
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class ballTargeting{ 

    private NetworkTableEntry tx;                         // Angle error (x) from LimeLight camera
    private NetworkTableEntry ty;                         // Angle error (y) from LimeLight camera
    private NetworkTableEntry ta;                         // Target area measurement from LimeLight camera
    private NetworkTableEntry tv;                         // Target valid indicator from Limelight camera
    // Shared variables
    public boolean ballValid;     // Indicate when the Limelight camera has found a target
    public boolean ballLocked;    // Indicate when the turret is centered on the target
    public double  ballRange;     // Range from robot to target (inches)
    //Private autoaim variables
    private double  turnSpeed;   
    private double  xError;
    private double  yError;
    private double  area;

    private static int BLUE_PIPELINE = 1;
    private static int RED_PIPELINE = 0;

    private static double Ball_ERROR = 0.5;           // Allowed aiming error in degrees
    private static double LOCK_ERROR = 1.0;
    private static double BALL_ROTATE_KP = 1.0 / 15.0;   // Proportional constant for turret rotate speed

    
    private static double CAMERA_HEIGHT = 0;            // Limelight height above ground (inches)
    private static double CAMERA_ANGLE  = 0;            // Limelight camera angle above horizontal (degrees)
    private static double TARGET_HEIGHT = 4.5;           // Center of target above ground (inches)
    private static double TARGET_HEIGHT_DELTA = TARGET_HEIGHT - CAMERA_HEIGHT;

    double translateX = (Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) / 2;        //X is forward Direction, Forward on Joystick is Y
    double translateY = (Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) / 2;

    public Drivetrain drive;
    public ColorSensor colorsensor;

    //motor controllors
    public WPI_TalonFX collector;
    

    public ballTargeting(){
        //collector = new WPI_TalonFX(config_hw.newFlywheelCollector);

        //set up networktables for limelight
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ball");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        


        // Establish initial values for variables we share
        ballValid   = false;
        ballLocked  = false;
        ballRange   = 0.0;


    }

    public double ballAim() {
        
        // Read the Limelight data from the Network Tables
        xError      = tx.getDouble(0.0);
        yError      = ty.getDouble(0.0);
        area        = ta.getDouble(0.0);
        ballValid = (tv.getDouble(0.0) != 0); // Convert the double output to boolean

        // Compute range to target.
        // Formula taken from https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
        ballRange = TARGET_HEIGHT_DELTA / Math.tan(Math.toRadians(CAMERA_ANGLE + yError));
        
        // Setting power based on the xError causes the turret to slow down as the error approaches 0
        // This prevents the turret from overshooting 0 and oscillating back and forth
        // KP is a scaling factor that we tested
        turnSpeed = xError * BALL_ROTATE_KP;
    
        if (Math.abs(xError) < LOCK_ERROR) {               // Turret is pointing at target (or no target)
          ballLocked = ballValid;                     // We are only locked when targetValid
        }
        else{
          ballLocked = false;
        }
    
        //post driver data to smart dashboard periodically
        SmartDashboard.putNumber("xerror in radians", Math.toRadians(xError));
        SmartDashboard.putNumber("Limelight BallX", xError);
        SmartDashboard.putNumber("Limelight BallY", yError);
        SmartDashboard.putNumber("Limelight BallArea", area);
        SmartDashboard.putNumber("Ball Range", ballRange);
        SmartDashboard.putBoolean("Ball Valid", ballValid);
        SmartDashboard.putBoolean("Ball Locked", ballLocked);
        SmartDashboard.putNumber("Turn Speed", turnSpeed);

            //x = 0 when the camera sees the target is in the center
        // Only allow the turret to track when commanded
        if (Math.abs(xError) < Ball_ERROR) {               // Turret is pointing at target (or no target)
            return 0.0; //Returns 0 if the turret is within the margin error
        }
        else {
            return turnSpeed;
        }
        }

      public void moveTowardsBall(){ 
        //x = 0 when the camera sees the target is in the center
        // Only allow the turret to track when commanded
        
        
        if (Math.abs(xError) < Ball_ERROR) {               // Turret is pointing at target (or no target)
            drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, drive.getGyroscopeRotation()));
        }
        else {
            drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0, ballAim() , drive.getGyroscopeRotation()));
        }
      }


      public void setLimelightAllianceColor(ALLIANCE_COLOR color){
        if (colorsensor.getCurrentBallColor() == ALLIANCE_COLOR.RED){
          NetworkTableInstance.getDefault().getTable("limelight ball").getEntry("pipeline").setNumber(RED_PIPELINE);
        }
        else {
          NetworkTableInstance.getDefault().getTable("limelight ball").getEntry("pipeline").setNumber(BLUE_PIPELINE);
        }
      }

      
    }
    

