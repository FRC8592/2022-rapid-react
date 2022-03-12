// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;

//import com.fasterxml.jackson.databind.deser.std.CollectionDeserializer.CollectionReferringAccumulator;




/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Our robot objects
  public XboxController driverController;
  public XboxController shooterController;
  public Drivetrain drive;
  public Autonomous autonomous;
  public Vision visionRing;
  public Vision visionBall;
  public Locality locality; 
  public Shooter shooter;
  public Collector collector;
  public CollectorArm arm;
  public ColorSensor colorSense;
  public Power powerMonitor;

  // Our alliance color (read from color sensor)
  private ColorSensor.BALL_COLOR allianceColor = ColorSensor.BALL_COLOR.NONE;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    driverController  = new XboxController(0); 
    shooterController = new XboxController(1);
    drive             = new Drivetrain();
    visionRing        = new Vision(Constants.LIMELIGHT_RING, Constants.RING_LOCK_ERROR,
                                   Constants.RING_CLOSE_ERROR, Constants.RING_CAMERA_HEIGHT,
                                   Constants.RING_CAMERA_ANGLE, Constants.RING_TARGET_HEIGHT,
                                   Constants.TURRET_ROTATE_KP);
    visionBall        = new Vision(Constants.LIMELIGHT_BALL, Constants.BALL_LOCK_ERROR,
                                   Constants.BALL_CLOSE_ERROR, Constants.BALL_CAMERA_HEIGHT,
                                   Constants.BALL_CAMERA_ANGLE, Constants.BALL_TARGET_HEIGHT,
                                   Constants.BALL_ROTATE_KP);
    locality          = new Locality(0, 0, drive);
    shooter           = new Shooter();
    collector         = new Collector();
    arm               = new CollectorArm();
    powerMonitor      = new Power();

    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }


  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    //
    // Create color sensor object here.  The color sensor will lock in the alliance color
    // upon creation.  The starting ball may not be inside the robot when it is first powered
    // up, so we do not want to instantiate the color sensor with other objects in robotInit()
    //
    // Once we have our alliance color, use it to activate the appropirate Limelight pipeline
    //
    colorSense     = new ColorSensor();
    allianceColor  = colorSense.getAllianceColor();
  
    NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline").setNumber(allianceColor.ordinal());

    autonomous = new Autonomous(drive);
  }

  @Override
  public void autonomousPeriodic(){
    colorSense.updateCurrentBallColor();
    visionBall.updateVision();
    visionRing.updateVision();
    locality.updatePosition(drive.getYaw(), visionRing);
    arm.update();
    collector.ballControl(arm, powerMonitor);
    shooter.computeFlywheelRPM(visionRing.distanceToTarget(), colorSense.isAllianceBallColor());
    powerMonitor.powerPeriodic();
  }
  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    // Zero the gyroscope to set our field-relative heading
    drive.zeroGyroscope();

    //
    // If we didn't run autonomus, initialize the color sensor and Limelight pipeline here
    // TODO: Initialization is not always run when enabling the robot.  We need to make sure we get the new color every time!
    //
    if (colorSense == null) {
      colorSense     = new ColorSensor();
      allianceColor  = colorSense.getAllianceColor();
    
      NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline").setNumber(allianceColor.ordinal());
    }
  }


  /** This function is called periodically during operator control. 
   * Aim turret
   * Run Flywheel and shooting mechanisms
   * Run drivetrain and collector
   * All done by driver
  */
  @Override
  public void teleopPeriodic(){
    double translateX;
    double translateY;
    double rotate;

    //
    // Call these methods on each update cycle to keep the robot running
    //
    colorSense.updateCurrentBallColor();
    visionBall.updateVision();
    visionRing.updateVision();
    locality.updatePosition(drive.getYaw(), visionRing);
    arm.update();
    collector.ballControl(arm, powerMonitor);
    shooter.computeFlywheelRPM(visionRing.distanceToTarget(), colorSense.isAllianceBallColor());
    powerMonitor.powerPeriodic();
 
    //
    // Current control scheme
    //   driverController (Left stick)   : translate
    //   driverController (Right stick)  : yaw
    //   driverController (Left trigger) : Auto aim at ring
    //   driverController (Right trigger): Shoot
    //   driverController (Left bumper)  : Auto ball fetch
    //   driverController (A button)     : Enter collect mode
    //   driverController (Y button)     : Exit collect mode
    //  
    //   shooterController (Left trigger) : Auto aim at ring
    //   shooterController (Right trigger): Shoot
    //   shooterController (Left bumper)  : Auto ball fetch
    //   shooterController (A button)     : Enter collect mode
    //   shooterController (Y button)     : Exit collect mode
    //   shooterController (L + R stick)  : Unjam

    //
    // Unjam the intake by reversing the staging and collector motors.  This function has top priority
    //
    if (shooterController.getLeftStickButtonPressed() &&  shooterController.getRightStickButtonPressed())
        collector.unjam(arm);
    else {
      //
      // Check to see if we have a ball for the opposite alliance loaded.  If so, get
      // rid of it by shooting it at low RPM.  The flywheel will automaticallly adjust
      // RPM in this condition, so here we only need to control the collector
      //
      if (!colorSense.isAllianceBallColor())
        collector.shoot();
      else {
        //
        // Enter collect mode
        //
        if ((driverController.getAButtonPressed()) || shooterController.getAButtonPressed())
          collector.enableCollectMode(arm, powerMonitor);
        //
        // Exit collect mode
        //
        else if ((driverController.getYButtonPressed()) || shooterController.getYButtonPressed())
          collector.disableCollectMode(arm, powerMonitor);
      
        //
        // Shoot ball
        //
        if ((driverController.getRightTriggerAxis() > 0.1 ) || (shooterController.getRightTriggerAxis() > 0.1 ))
          collector.shoot();
      }
    }

    //
    // Read gamepad controls for drivetrain and scale control values
    //
    rotate     = (driverController.getRightX() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) * ConfigRun.ROTATE_POWER;  // Right joystick
    translateX = (driverController.getLeftY()  * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * ConfigRun.TRANSLATE_POWER;        // X is forward Direction, Forward on Joystick is Y
    translateY = (driverController.getLeftX()  * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * ConfigRun.TRANSLATE_POWER;
  
    //
    // Activate ring targetting.  Robot translate controls are functional while targetting
    //
    if ((driverController.getLeftTriggerAxis() > 0.1 ) || (shooterController.getLeftTriggerAxis() > 0.1 )) {
      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-joystickDeadband(translateX), -joystickDeadband(translateY),
                  -visionRing.turnRobot() , drive.getGyroscopeRotation()));
    }

    //
    // Activate ball (cargo) targetting and fetching.  Robot motion controls are unavailable while targetting balls
    //
    else if (driverController.getLeftBumperPressed() || shooterController.getLeftBumperPressed()) {
      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(visionBall.moveTowardsTarget(), 0,
                  visionBall.turnRobot(), Rotation2d.fromDegrees(0)));
    }

    //
    // Normal teleop drive
    //
    else {
      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-joystickDeadband(translateX), -joystickDeadband(translateY),
        -joystickDeadband(rotate), drive.getGyroscopeRotation()));     //Inverted due to Robot Directions being the opposite of controller directions
    }
  }


  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }


  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  //
  // Implement a joystick deadband
  //
  public double joystickDeadband(double inputJoystick) {
    if (Math.abs(inputJoystick) < ConfigRun.JOYSTICK_DEADBAND) {
      return 0;
    } else {
      return inputJoystick;
    }
  }

}

