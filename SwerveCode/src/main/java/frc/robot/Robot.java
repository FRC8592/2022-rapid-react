// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
  public AutoDrive locality;
  public Shooter shooter;
  public Collector collector;
  public CollectorArmMM arm;
  public Climber climber;
  public ColorSensor colorSense;
  public Power powerMonitor;
  public Timer timer;
  public AutoWaypoint autoWaypoint;

  // Our alliance color (read from color sensor)
  private ColorSensor.BALL_COLOR allianceColor = ColorSensor.BALL_COLOR.NONE;

  // Indicate if Autonmous has run. If not, we have some things to initialize in
  // teleopInit()
  private boolean AutonomousHasRun = false;

  // Variables for simple autonomous
  private enum AutoState {
    TURN, SHOOT, DRIVE
  };

  AutoState autoState = AutoState.TURN;
  private double autoStateTime;

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

    driverController = new XboxController(0);
    shooterController = new XboxController(1);
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

    // Turn all of our blindingly bright lights off until neeeded.
    powerMonitor.relayOff();
    NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("ledMode")
        .setNumber(Constants.LIMELIGHT_LIGHT.FORCE_OFF.ordinal());
    NetworkTableInstance.getDefault().getTable("limelight-ring").getEntry("ledMode")
        .setNumber(Constants.LIMELIGHT_LIGHT.FORCE_OFF.ordinal());

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

    autoWaypoint = new AutoWaypoint(locality,drive, collector, shooter, visionRing, visionBall);
    if(ConfigRun.WAYPOINT){
      autoWaypoint.addWaypoint(new Waypoint(-2, 0, 0.5, false, true, false, new Timer()));
      autoWaypoint.addWaypoint(new Waypoint(-2, -1.5, 0.1, false, false, false, new Timer()));
      autoWaypoint.addWaypoint(new Waypoint(0, 0, 0.1, true, false, true, new Timer()));
    }

    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    //
    // Create color sensor object here. The color sensor will lock in the alliance
    // color
    // upon creation. The starting ball may not be inside the robot when it is first
    // powered
    // up, so we do not want to instantiate the color sensor with other objects in
    // robotInit()
    //
    // Once we have our alliance color, use it to activate the appropirate Limelight
    // pipeline
    //
    colorSense = new ColorSensor();
    allianceColor = colorSense.getAllianceColor(); // Determine alliance color based on inserted ball
    timer.start();

    // Set up the proper ball-seeking pipeline for our alliance color
    NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline")
        .setNumber(allianceColor.ordinal());

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
    autoState = AutoState.TURN;
  }
  /**
   * Simple 2-ball autonomous routine
   */
  public void autonomousPeriodic() {
    colorSense.updateCurrentBallColor();
    visionBall.updateVision();
    visionRing.updateVision();
    locality.updatePosition(drive.getYaw(), visionRing);
    arm.update();
    collector.ballControl(arm, shooter, visionRing, powerMonitor);
    shooter.computeFlywheelRPM(visionRing.distanceToTarget(), colorSense.isAllianceBallColor());
    powerMonitor.powerPeriodic();
    // Turn to ring, then shoot, then drive backwards until we see the ring being 13
    // feet away
    // decide state changes
    if (ConfigRun.WAYPOINT) {
        autoWaypoint.runWaypoint();
    }
    else {
      switch (autoState) {
        case TURN:
          if (visionRing.targetLocked) {
            autoState = AutoState.DRIVE;
            autoStateTime = timer.get() + 1.0;
          }
          break;
        case SHOOT:

          break;
        case DRIVE:
          if (collector.getCollectorState() == Collector.CollectorState.TWO_BALLS) {
            autoState = AutoState.SHOOT;

          }
          break;
      }

      SmartDashboard.putString("autoState", autoState.name());

      // execute current state
      switch (autoState) {
        case SHOOT:
          collector.shoot();
          drive.drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, visionRing.turnRobot(), drive.getGyroscopeRotation()));
          break;
        case TURN:
          drive.drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, visionRing.turnRobot(), drive.getGyroscopeRotation()));
          collector.enableCollectMode(arm, powerMonitor);
          break;
        case DRIVE:
          drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(visionBall.moveTowardsTarget(), 0.0, visionBall.turnRobot(),
              Rotation2d.fromDegrees(0)));
          collector.enableCollectMode(arm, powerMonitor);
          break;
      }
    }
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {

    //
    // If we haven't run autonomous, do most of the autonomous initialization here
    //
    if (!AutonomousHasRun) {
      // Create color sensor object here.
      colorSense = new ColorSensor();
      allianceColor = colorSense.getAllianceColor(); // Determine alliance color based on inserted ball

      // Set up the proper ball-seeking pipeline for our alliance color
      NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline")
          .setNumber(allianceColor.ordinal());

      // Zero the gyroscope for field-relative drive
      drive.zeroGyroscope();
    }

    powerMonitor.relayOn();
    NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("ledMode").setNumber(Constants.LIMELIGHT_LIGHT.PIPELINE_MODE.ordinal());
    NetworkTableInstance.getDefault().getTable("limelight-ring").getEntry("ledMode").setNumber(Constants.LIMELIGHT_LIGHT.PIPELINE_MODE.ordinal());
    
    collector.reset();
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
    collector.ballControl(arm, shooter, visionRing, powerMonitor);
    shooter.computeFlywheelRPM(visionRing.distanceToTarget(), colorSense.isAllianceBallColor());
    powerMonitor.powerPeriodic();

    //
    // Current control scheme
    // driverController (Left stick) : translate
    // driverController (Right stick) : yaw
    // driverController (Left trigger) : Auto aim at ring
    // driverController (Right trigger): Shoot
    // driverController (Left bumper) : Auto ball fetch
    // driverController (A button) : Enter collect mode
    // driverController (Y button) : Exit collect mode
    // driverController (BACK + X) : Reset current heading to 0
    //
    // shooterController (Left trigger) : Auto aim at ring
    // shooterController (Right trigger) : Shoot
    // shooterController (Left bumper) : Auto ball fetch
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
      // Check to see if we have a ball for the opposite alliance loaded. If so, get
      // rid of it by shooting it at low RPM. The flywheel will automaticallly adjust
      // RPM in this condition, so here we only need to control the collector
      //
      if (!colorSense.isAllianceBallColor())
        collector.forceShoot();
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
    }

    //
    // Reset gyroscope zero for field-relative driving
    //
    if (driverController.getXButton() && driverController.getBackButton()) {
      System.out.println("Override - zeroing gyroscope");
      drive.zeroGyroscope();
    }

    //
    // Force Blue alliance
    //
    if (shooterController.getXButtonPressed() && shooterController.getBackButton()) {
      colorSense.forceBlueAlliance();
      allianceColor = colorSense.getAllianceColor();
      NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline")
          .setNumber(allianceColor.ordinal());
    }

    //
    // Force Red alliance
    //
    if (shooterController.getBButtonPressed() && shooterController.getBackButton()) {
      colorSense.forceRedAlliance();
      allianceColor = colorSense.getAllianceColor();
      NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline")
          .setNumber(allianceColor.ordinal());
    }

    //
    // Temporary control for climber
    //  moves arm up and down, checks that arm doesn't overextend
    //climber.liftPeriodic(joystickDeadband(shooterController.getRightY()));
    climber.moveLift(joystickDeadband(shooterController.getRightY()));

    //
    // Read gamepad controls for drivetrain and scale control values
    //
    rotate = (driverController.getRightX() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
        * ConfigRun.ROTATE_POWER; // Right joystick
    translateX = (driverController.getLeftY() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * ConfigRun.TRANSLATE_POWER; // X
                                                                                                                        // is
                                                                                                                        // forward
                                                                                                                        // Direction,
                                                                                                                        // Forward
                                                                                                                        // on
                                                                                                                        // Joystick
                                                                                                                        // is
                                                                                                                        // Y
    translateY = (driverController.getLeftX() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * ConfigRun.TRANSLATE_POWER;

    //
    // Activate ring targetting. Robot translate controls are functional while
    // targetting
    //
    if ((driverController.getLeftTriggerAxis() > 0.1) || (shooterController.getLeftTriggerAxis() > 0.1)) {
      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-joystickDeadband(translateX), -joystickDeadband(translateY),
          visionRing.turnRobot(), drive.getGyroscopeRotation()));
    }

    //
    // Activate ball (cargo) targetting and fetching. Robot motion controls are
    // unavailable while targetting balls
    //
    else if (driverController.getLeftBumper() || shooterController.getLeftBumper()) {
      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(visionBall.moveTowardsTarget(), 0,
          visionBall.turnRobot(), Rotation2d.fromDegrees(0)));
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
    colorSense = null;

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
    // Prevent possible(?) timeouts from occuring by sending commands to the motor
    // continuously
    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, drive.getGyroscopeRotation()));
    // Pulls arm down until motor current peaks, current peaks = arm is parked
    

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
