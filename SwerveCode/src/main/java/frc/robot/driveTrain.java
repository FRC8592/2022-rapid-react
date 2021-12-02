package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController; //this puts in the xbox contoller stuff

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import frc.robot.Constants;

import static frc.robot.Constants.*;

public class driveTrain {
  // Constants
  private static final double DRIVE_POWER = 1.0;    // Forward/reverse power scaling
  private static final double TURN_POWER  = 0.6;    // Turning power scaling
  private static final double TURN_IN_PLACE_POWER  = 0.45;    // Turning power scaling
  private static final double RAMP_TIME   = 0.25;    // Smooth application of motor power

  // Motor controllers
  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  // Motor groups
	SpeedControllerGroup leftDrive;
	SpeedControllerGroup rightDrive;
	  
	// Differential drive class
  DifferentialDrive robotDrive;


  /**
   * Initialize drivetrain
   */
  public driveTrain(){
    // Create motor objects
    
    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // FIXME Setup motor configuration
    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            /*tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
                    */
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

  }

/**Drives robot: forwards, backwards, turning*/
  public void driveTrainPeriodic(XboxController driveTrainController){
    double  forward;
    double  reverse;
    double  throttle;
    double  turn;
    boolean reverseControl;
    boolean curveOff;

    // Read gamepad controls
    forward = driveTrainController.getTriggerAxis(GenericHID.Hand.kRight);  // Right trigger
    reverse = driveTrainController.getTriggerAxis(GenericHID.Hand.kLeft);   // Left Trigger
    turn    = driveTrainController.getX(GenericHID.Hand.kLeft);             // Left joystick
    //
    reverseControl = driveTrainController.getBButton();                      // B button
    curveOff       = driveTrainController.getBumper(GenericHID.Hand.kRight); // Right bumper
  
    // Combine and scale inputs
    throttle = (-forward + reverse) * DRIVE_POWER;

    // Apply a different power curve for turn-in-place
    if (!curveOff)
      turn = turn * TURN_POWER;
    else
      turn = turn * TURN_IN_PLACE_POWER;

    // If reverseControl is being pressed, invert all inputs so the robot can be driven backwards
    if (reverseControl) {
      throttle = -throttle;
    }
    
    // Send controls to the robot drive system
    robotDrive.curvatureDrive(throttle, turn, curveOff);
  }

  public void autoDrive(){
    robotDrive.curvatureDrive(0.3, 0, false);
  }

  public void driveStop(){
    robotDrive.curvatureDrive(0, 0, false);
  }

}
