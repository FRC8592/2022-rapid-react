// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController; //this puts in the xbox contoller stuff
import frc.robot.Shooter;

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
  //this defines "driverController" as an object
  public XboxController driverController;
  public XboxController shooterController;

  public Drivetrain drive;
  public Autonomous autonomous;
  public Vision vision;
  public Locality locality; 
  public Shooter shooter;
  public Collector collector;
  public ColorSensor color;

  
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
    vision            = new Vision();
    locality          = new Locality(0, 0);
    shooter           = new Shooter();
    color             = new ColorSensor();

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

        autonomous = new Autonomous(drive);
    }
  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    drive.zeroGyroscope();
    //Create the primary controller object
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
    
    locality.updatePosition(drive.getYaw(), vision);

    // Read gamepad controls
    rotate     = (driverController.getRightX() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) * ConfigRun.ROTATE_POWER;            // Right joystick
    translateX = (driverController.getLeftY() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * ConfigRun.TRANSLATE_POWER;             //X is forward Direction, Forward on Joystick is Y
    translateY = (driverController.getLeftX() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * ConfigRun.TRANSLATE_POWER;

    
  
    if(driverController.getRightBumper() == true){
      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-joystickDeadband(translateX),-joystickDeadband(translateY), -vision.autoAim() , drive.getGyroscopeRotation()));     //Inverted due to Robot Directions being the opposite of controller directions 
    } else {
    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-joystickDeadband(translateX),-joystickDeadband(translateY), -joystickDeadband(rotate), drive.getGyroscopeRotation()));     //Inverted due to Robot Directions being the opposite of controller directions
    }

    //this makes sure that when the driver pushes the A button they can control the shooter directly, if not this runs the ball control
    if(shooterController.getAButton()){
      shooter.testshooter(shooterController);
    }else{
      collector.ballControl();
    }

    SmartDashboard.putNumber("Rotate", rotate);
    color.getColors();
    if(color.getProximity() > 200){
      color.updateCurrentBallColor();
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


    public double joystickDeadband(double inputJoystick) {
        if (Math.abs(inputJoystick) < ConfigRun.JOYSTICK_DEADBAND) {
            return 0;
        } else {
            return inputJoystick;
        }
    }

}
