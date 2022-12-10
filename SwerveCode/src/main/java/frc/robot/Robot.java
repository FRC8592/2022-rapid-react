// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.common.ConfigRun;
import frc.common.Constants;
import frc.common.Data;
import frc.common.Waypoint;
import frc.robot.autonomous.AutoDrive;
import frc.robot.autonomous.AutoWaypoint;
import frc.robot.autonomous.Autonomous;
import frc.robot.hardware.LEDstrips;
import frc.robot.hardware.Power;
import frc.robot.modules.*;

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

  public static Field2d FIELD = new Field2d();
  public static Data DATA = new Data();

  // Our robot objects
  public XboxController driverController;
  public XboxController shooterController;
  public GenericHID controlPanel;
  public Drivetrain drive;
  public Autonomous autonomous;
  public Vision visionRing;
  public Vision visionBall;
  public AutoDrive locality;
  public Shooter shooter;
  public Collector collector;
  public CollectorArmMM arm;
  public Climber climber;
  public Power powerMonitor;
  public Timer timer;
  public AutoWaypoint autoWaypoint;
  public frc.robot.hardware.LEDstrips LEDstrips;

  private ModuleList runningModules;

  // Toggle for fast/slow mode
  private boolean fastMode;
  private boolean slowModeToggle;

  // Toggle to lock flywheel speed\
  private boolean flywheelLock;
  private boolean flywheelLockToggle;

  // Our alliance color
  private DriverStation.Alliance allianceColor;

  // Indicate if Autonmous has run. If not, we have some things to initialize in
  // teleopInit()
  private boolean AutonomousHasRun = false;


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    fastMode = true;
    runningModules = new ModuleList();

    driverController = new XboxController(0);
    shooterController = new XboxController(1);
    controlPanel = new GenericHID(2);
    drive = new Drivetrain();
    visionRing = new Vision(Constants.LIMELIGHT_RING, Constants.RING_LOCK_ERROR,
        Constants.RING_CLOSE_ERROR, Constants.RING_CAMERA_HEIGHT,
        Constants.RING_CAMERA_ANGLE, Constants.RING_TARGET_HEIGHT,
        Constants.TURRET_ROTATE_KP, Constants.TURRET_ROTATE_KI,
        Constants.TURRET_ROTATE_KD);
    visionBall = new Vision(Constants.LIMELIGHT_BALL, Constants.BALL_LOCK_ERROR,
        Constants.BALL_CLOSE_ERROR, Constants.BALL_CAMERA_HEIGHT,
        Constants.BALL_CAMERA_ANGLE, Constants.BALL_TARGET_HEIGHT,
        Constants.BALL_ROTATE_KP, Constants.BALL_ROTATE_KI,
        Constants.BALL_ROTATE_KD);
    locality = new AutoDrive(0, 0, drive);
    shooter = new Shooter();
    collector = new Collector();
    arm = new CollectorArmMM();
    climber = new Climber();
    powerMonitor = new Power();
    timer = new Timer();
    autonomous = new Autonomous(drive);
    LEDstrips = new LEDstrips();
    

    // Turn all of our blindingly bright lights off until neeeded.
    powerMonitor.relayOff();
    NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("ledMode")
        .setNumber(Constants.LIMELIGHT_LIGHT.FORCE_OFF.ordinal());
    NetworkTableInstance.getDefault().getTable("limelight-ring").getEntry("ledMode")
        .setNumber(Constants.LIMELIGHT_LIGHT.FORCE_OFF.ordinal());

    SmartDashboard.putData(FIELD);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
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
    autonomous.initialize();
    autonomous.resetAuto();
    autoWaypoint = new AutoWaypoint(locality,drive, collector, shooter, visionRing, visionBall);
    if(ConfigRun.WAYPOINT){
      autoWaypoint.addWaypoint(new Waypoint(-2, 0, 0.5, false, true, false, new Timer()));
      autoWaypoint.addWaypoint(new Waypoint(-2, -1.5, 0.1, false, false, false, new Timer()));
      autoWaypoint.addWaypoint(new Waypoint(0, 0, 0.1, true, false, true, new Timer()));
    }

    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);

    //
    // Get our alliance color from the field control system
    //
    allianceColor = DriverStation.getAlliance();
    // allianceColor = DriverStation.Alliance.Red;

    // Ensure we are in fast mode or the flywheel won't operate
    fastMode     = true;
    flywheelLock = false;
    flywheelLockToggle = false;

    //
    // Set up the proper ball-seeking pipeline for our alliance color
    //
    // TODO: Clean this up so we're not assuming that Red = 0 and Blue = 1
    //
    if (allianceColor == DriverStation.Alliance.Red)
      NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline")
                          .setNumber(0);
    else
      NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline")
                          .setNumber(1);

    // Allow limelight lights to turn back on
    powerMonitor.relayOn();
    NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("ledMode")
        .setNumber(Constants.LIMELIGHT_LIGHT.PIPELINE_MODE.ordinal());
    NetworkTableInstance.getDefault().getTable("limelight-ring").getEntry("ledMode")
        .setNumber(Constants.LIMELIGHT_LIGHT.PIPELINE_MODE.ordinal());

    // Zero the gyroscope for field-relative drive
    drive.zeroGyroscope();

    // Indicate to teleop that autonomous has run
    AutonomousHasRun = true;
  }


  /**
   * Simple 2-ball autonomous routine
   */
  public void autonomousPeriodic() {
    // autonomous.autonomousPeriodic(visionBall, visionRing, arm, locality, collector, shooter, powerMonitor, drive);
    autonomous.autonomousPeriodic();
    //SmartDashboard.putNumber("Gyroscope Value", drive.getAutoHeading());
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
    LEDstrips.upAndDown();
    //
    // If we haven't run autonomous, do most of the autonomous initialization here
    //
    if (!AutonomousHasRun) {

      //
      // Get our alliance color from the field control system
      //
      //allianceColor = DriverStation.getAlliance();

      //
      // Set up the proper ball-seeking pipeline for our alliance color
      //
      // TODO: Clean this up so we're not assuming that Red = 0 and Blue = 1
      //
      if (allianceColor == DriverStation.Alliance.Red)
        NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline")
                            .setNumber(0);
      else
        NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline")
                            .setNumber(1);

      // Zero the gyroscope for field-relative drive
      drive.zeroGyroscope();
    }

    // Always start in fast mode
    fastMode     = true;
    flywheelLock = false;

    //slow mode is off
    slowModeToggle = false;

    // Turn on lights
    powerMonitor.relayOn();
    NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("ledMode").setNumber(Constants.LIMELIGHT_LIGHT.PIPELINE_MODE.ordinal());
    NetworkTableInstance.getDefault().getTable("limelight-ring").getEntry("ledMode").setNumber(Constants.LIMELIGHT_LIGHT.PIPELINE_MODE.ordinal());
    
    collector.reset();
    arm.reset();
    shooter.reset();
    visionRing.reset();
    visionBall.reset();
    climber.reset();
  }

  /**
   * This function is called periodically during operator control.
   * Aim robot
   * Run Flywheel and shooting mechanisms
   * Run drivetrain and collector
   * All done by driver
   */
  @Override
  public void teleopPeriodic() {
    double rotatePower;
    double translatePower;
    double translateX;
    double translateY;
    double rotate;
    SmartDashboard.putNumber("Gyroscope Value2", drive.getAutoHeading());
    //
    // Call these methods on each update cycle to keep the robot running
    //
    visionBall.updateVision();
    visionRing.updateVision();
    locality.updatePosition(drive.getYaw(), visionRing);
    arm.update();
    collector.ballControl(arm, shooter, visionRing, powerMonitor);
    shooter.computeFlywheelRPM(visionRing.distanceToTarget(), fastMode, flywheelLock);
    powerMonitor.powerPeriodic();
    
    //
    // Current control scheme
    // driverController (Left stick) : translate
    // driverController (Right stick) : yaw
    // driverController (Left trigger) : Auto aim at ring
    // driverController (Right trigger): Shoot
    // driverController (Left bumper) : Auto ball fetch
    // driverController (Right bumper) : Toggle fast / slow mode
    // driverController (A button) : Enter collect mode
    // driverController (Y button) : Exit collect mode
    // driverController (BACK + X) : Reset current heading to 0
    //
    // shooterController (Left trigger) : Auto aim at ring
    // shooterController (Right trigger) : Shoot
    // shooterController (Left bumper) : Auto ball fetch
    // shooterController (Right bumper) : toggle flywheel lock mode
    // shooterController (Right stick Y) : Lift up and down
    // shooterController (A button) : Enter collect mode
    // shooterController (Y button) : Exit collect mode
    // shooterController (L + R stick) : Unjam
    // shooterController (BACK + blue) : Force blue alliance
    // shooterController (BACK + red) : Force red alliance
    // shooterController (BACK Right bumper) : Shoot without lock on ring
    
    //
    // Unjam the intake by reversing the staging and collector motors. This function
    // has top priority
    //
    if (shooterController.getLeftStickButton() && shooterController.getRightStickButton())
    collector.unjam(arm);
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
      // Shoot ball with aiming automation disabled
      //
      if (shooterController.getRightBumper() && shooterController.getBackButton())
      collector.forceShoot();
      
      //
      // Shoot ball
      //
      else if ((driverController.getRightTriggerAxis() > 0.1) || (shooterController.getRightTriggerAxis() > 0.1))
      collector.shoot();
    }
    
    
    //
    // Reset gyroscope zero for field-relative driving
    //
    if (driverController.getXButton() && driverController.getBackButton() || controlPanel.getRawButtonPressed(1)) {
      //System.out.println("Override - zeroing gyroscope");
      drive.zeroGyroscope();
    }
    
    //
    // Force Blue alliance
    //
    if (shooterController.getXButtonPressed() && shooterController.getBackButton() || controlPanel.getRawButtonPressed(2)) {
      allianceColor = DriverStation.Alliance.Blue;
      NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline")
      .setNumber(1);
    }
    
    //
    // Force Red alliance
    //
    if (shooterController.getBButtonPressed() && shooterController.getBackButton() || controlPanel.getRawButtonPressed(3)) {
      allianceColor = DriverStation.Alliance.Red;
      NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline")
      .setNumber(0);
    }
    
    //
    // Toggle fast/slow mode
    //
    // *** FLYWHEEL IS FORCED TO 0 RPM in Slow Mode
    //
    if (driverController.getRightBumperPressed()){
      slowModeToggle = ! slowModeToggle;
    }
    fastMode = ! slowModeToggle && !controlPanel.getRawButton(7);  
    SmartDashboard.putBoolean("Fast Mode", fastMode);
    //
    // Toggle flywheel lock mode
    //
    if (shooterController.getRightBumperPressed()){
      flywheelLockToggle = ! flywheelLockToggle;
    }
    flywheelLock = flywheelLockToggle || controlPanel.getRawButton(6);
    SmartDashboard.putBoolean("Fly Wheel Lock", flywheelLock);
    
    
    //
    // Control for climber
    // Moves arm up and down, checks that arm doesn't overextend
    //
    //climber.liftPeriodic(joystickDeadband(shooterController.getRightY()));
    climber.moveLift(joystickDeadband(shooterController.getRightY()));

    //
    // Select driver power settings based on fast/slow mode
    //
    if (fastMode) {
      rotatePower    = ConfigRun.ROTATE_POWER_FAST;
      translatePower = ConfigRun.TRANSLATE_POWER_FAST;
    }
    else {
      rotatePower    = ConfigRun.ROTATE_POWER_SLOW;
      translatePower = ConfigRun.TRANSLATE_POWER_SLOW;
      collector.disableCollectMode(arm, powerMonitor);
    }

    //
    // Display robot heading
    //
    SmartDashboard.putNumber("Heading", 360 - drive.getGyroscopeRotation().getDegrees());

    //
    // Read gamepad controls for drivetrain and scale control values
    //
    rotate = (driverController.getRightX() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
        * rotatePower; // Right joystick
    translateX = (driverController.getLeftY() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * translatePower; // X
                                                                                                                        // is
                                                                                                                        // forward
                                                                                                                        // Direction,
                                                                                                                        // Forward
                                                                                                                        // on
                                                                                                                        // Joystick
                                                                                                                        // is
                                                                                                                        // Y
    translateY = (driverController.getLeftX() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * translatePower;

    //
    // Activate ring targetting. Robot translate controls are functional while
    // targetting
    //
    if ((driverController.getLeftTriggerAxis() > 0.1) || (shooterController.getLeftTriggerAxis() > 0.1)) {
      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-joystickDeadband(translateX), -joystickDeadband(translateY),
          visionRing.turnRobot(-1), drive.getGyroscopeRotation()));
    }

    //
    // Activate ball (cargo) targetting and fetching. Robot motion controls are
    // unavailable while targetting balls
    //
    else if (driverController.getLeftBumper() || shooterController.getLeftBumper()) {
      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(visionBall.moveTowardsTarget(ConfigRun.TARGET_LOCKED_SPEED, ConfigRun.TARGET_CLOSE_SPEED), 0,
          visionBall.turnRobot(-1), Rotation2d.fromDegrees(0)));
    }

    //
    // Normal teleop drive
    //
    else {
      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-joystickDeadband(translateX), -joystickDeadband(translateY),
          -joystickDeadband(rotate), drive.getGyroscopeRotation())); // Inverted due to Robot Directions being the
                                                                     // opposite of controller directions
    }
    drive.getCurrentPos();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

    // Turn off lights when not operating. Make more friends and fewer enemies this
    // way.
    powerMonitor.relayOff();
    NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("ledMode")
        .setNumber(Constants.LIMELIGHT_LIGHT.FORCE_OFF.ordinal());
    NetworkTableInstance.getDefault().getTable("limelight-ring").getEntry("ledMode")
        .setNumber(Constants.LIMELIGHT_LIGHT.FORCE_OFF.ordinal());
    
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    LEDstrips.upAndDown();
    // Prevent possible(?) timeouts from occuring by sending commands to the motor
    // continuously
    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drive.getGyroscopeRotation()));
    SmartDashboard.putNumber("Yaw value", locality.getYaw());
    SmartDashboard.putNumber("Position Y (inch)", locality.inchesToMeters(locality.getY()));
    SmartDashboard.putNumber("Position X (inch)", locality.inchesToMeters(locality.getX()));
    SmartDashboard.putNumber("Position Valid", locality.isGood() ? 1.0: 0.0);
    SmartDashboard.putBoolean("limitSwitch", arm.outputLimitSwitch());

    drive.getCurrentPos();
    // Pulls arm down until motor current peaks, current peaks = arm is parked
    

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    climber.reset();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    climber.pullArmDown();
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
