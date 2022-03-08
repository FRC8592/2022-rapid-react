//////////////////////////////////////////////////////////////////////////////
// Control the flywheel shooter rotation and when we launch a ball
//////////////////////////////////////////////////////////////////////////////

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class Shooter{ 
    // The collector is instantiated in this class
    Collector collector;

    // Paired motor controllers to drive the launch wheel
    public WPI_TalonFX flyWheelRight;
    public WPI_TalonFX flyWheelLeft;

    // Flywheel status
    private boolean flywheelReady = false;

    // Internal state
    public enum ShooterState{AUTONOMOUS,SHOOT}
    private ShooterState shooterState;


    //
    // Constructor to instantiate the Collector object and the flywheel motors
    //
    public Shooter(){
        // Instantiate our collector object
        collector = new Collector();

        // Instantiate the launch motors and configure them to factory default settings
        flyWheelLeft  = new WPI_TalonFX(Constants.newFlywheelLeft);
        flyWheelRight = new WPI_TalonFX(Constants.newFlywheelRight);
        flyWheelLeft.configFactoryDefault();
        flyWheelRight.configFactoryDefault();

        // Configure the left flywheel motor to follow the right flywheel motor
        flyWheelLeft.follow(flyWheelRight);
        flyWheelLeft.setInverted(InvertType.OpposeMaster);

        // Settings for flywheel PID constant velocity mode
        flyWheelRight.configVoltageCompSaturation(Constants.FLYWHEEL_VOLTAGE);
        flyWheelRight.enableVoltageCompensation(true);   // Enable voltage compensation
        flyWheelRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        flyWheelRight.config_kP(0, Constants.FLYWHEEL_P);
        flyWheelRight.config_kI(0, Constants.FLYWHEEL_I);
        flyWheelRight.config_kD(0, Constants.FLYWHEEL_D);
        flyWheelRight.config_kF(0, Constants.FLYWHEEL_F);
        flyWheelRight.configClosedloopRamp(1);
        flyWheelRight.set(ControlMode.Velocity, 0);
    }


    /**
     * Converts an RPM value to the ticks/100ms velocity metric used by the
     * Falcon 500 motor
     * 
     * @param rpm Rotational velocity in Revolutions Per Minute (RPM)
     * @return Encoder ticks per 100msec
     */
    private double rpmToFalcon(double rpm) {
        return rpm * Constants.RPM_TO_TICKS_MS;
    }


    /**
     * Converts the internal Falcon 500 velocity measurement of ticks per 100msec
     * into Revolutions Per Minute (RPM)
     * 
     * @param falcon Rotational velocity in ticks/100msec. This is usually read from
     *               the controller
     * @return Rotational velocity in RPM
     */
    private double falconToRpm(double falconTps) {
        return falconTps / Constants.RPM_TO_TICKS_MS;
    }


    /**
     * Set the desired flywheel speed (RPM)
     * Read the actual flywheel speed and compute the error
     * Update flywheelReady status based on the RPM error
     *
     * @param flywheelRpmSet Desired flywheel setpoint in RPM
     */
    private void updateFlywheel(double flywheelRpmSet) {
        double flywheelRpmActual;
        double flywheelRpmError;

        flyWheelRight.set(ControlMode.Velocity, rpmToFalcon(flywheelRpmSet));
        flywheelRpmActual = falconToRpm(flyWheelRight.getSelectedSensorVelocity());
        flywheelRpmError  = Math.abs(flywheelRpmSet - flywheelRpmActual);

        if (flywheelRpmError <= Constants.RPM_MAX_ERROR)
            flywheelReady = true;
        else
            flywheelReady = false;

        // Post flywheel parameters to Smart Dashboard
        SmartDashboard.putNumber("Flywheel Setpoint", flywheelRpmSet);
        SmartDashboard.putNumber("Flywheel Actual", flywheelRpmActual);
        SmartDashboard.putBoolean("Flywheel Ready", flywheelReady);
    }
    
    
    /**
     * This method will eventually compute the correct flywheel speed based on
     * range to target and update the flywheel controller with desired speed.
     * 
     * It currently just reads a desired flywheel speed from the Smart
     * Dashboard.
     * 
     * This method should be called on every robot update to keep flywheel
     * parameters updated.
     * 
     * @param range Range to target (units?)
     */
    private void computeFlywheelRPM(double range) {
        double flywheelRpmSet;

        flywheelRpmSet = SmartDashboard.getNumber("Flywheel Set (RPM)", Constants.STARTING_FLYWHEEL_SPEED);
        updateFlywheel(flywheelRpmSet);     // Send desired RPM to flywheel controller

        return;
    }


    /**
     * 
     * @return
     */
    public ShooterState determineShooterState(){
        Collector.CollectorState collectorState = collector.determineCollectorState();

        if(collectorState != Collector.CollectorState.NO_BALLS_LOADED){
            shooterState = ShooterState.SHOOT;
        }else{
            shooterState = ShooterState.AUTONOMOUS;
        }

        SmartDashboard.putString("State", shooterState.toString());
        return shooterState;
    }


    /**
     * 
     * @param shootController
     */
    public void collectorDriverControl(XboxController shootController){
        // Update our flywheel velocity and status
        computeFlywheelRPM(0);  // TODO: Send the range to target to this method

        switch (determineShooterState()) {
            case SHOOT:
                if(shootController.getBButton()){
                    manualControl(); // TODO: Check flywheel RPM and aiming status
                    
                }else{
                    collector.ballControl();
                }
                break;

            case AUTONOMOUS:
                collector.ballControl();
                break;
        
        }
    }


    /**
     * 
     */
    public void manualControl(){
       //collector.intakeAllRun();
        SmartDashboard.putNumber("Running Manual", 1);
    }
}