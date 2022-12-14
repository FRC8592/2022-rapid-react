//////////////////////////////////////////////////////////////////////////////////////////////////////
// Manage the collector arm with Motion Magic
/////////////////////////////////////////////////////////////////////////////////////////////////////
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;


public class Climber {
    // Configuration constants
    private static final int    RAISE_PID_SLOT = 0;
    private static final int    LOWER_PID_SLOT = 1;

    // Object variables
    private WPI_TalonFX liftMotorRight;
    private WPI_TalonFX liftMotorLeft;
    
    // Internal global variables
    private boolean rightArmParked = false;
    private boolean leftArmParked  = false;


    // Configure the lift motors
    public Climber () {

        // Create the lift motor objects and clear configuration to factory defaults
        liftMotorRight = new WPI_TalonFX(Constants.LIFT_RIGHT_CAN);
        liftMotorLeft  = new WPI_TalonFX(Constants.LIFT_LEFT_CAN);
        liftMotorRight.configFactoryDefault();
        liftMotorLeft.configFactoryDefault();

        // Configure voltage compensation and current limiting to both motors
        liftMotorRight.configVoltageCompSaturation(Constants.LIFT_VOLTAGE);
        liftMotorRight.enableVoltageCompensation(true);   // Enable voltage compensation
        liftMotorRight.configSupplyCurrentLimit(Constants.LIFT_CURRENT_LIMIT);
        liftMotorLeft.configVoltageCompSaturation(Constants.LIFT_VOLTAGE);
        liftMotorLeft.enableVoltageCompensation(true);    // Enable voltage compensation
        liftMotorLeft.configSupplyCurrentLimit(Constants.LIFT_CURRENT_LIMIT);

        // Reduce deadband so that we can use small power levels (The follower [see below] will automatically disable its deadband)
        liftMotorRight.configNeutralDeadband(Constants.LIFT_DEADBAND);

        // Set motors to brake mode when idle
        liftMotorLeft.setNeutralMode(NeutralMode.Brake);
        liftMotorRight.setNeutralMode(NeutralMode.Brake);

        // liftMotorLeft.configNominalOutputForward(0);
        // liftMotorLeft.configNominalOutputReverse(0);
        // liftMotorLeft.configPeakOutputForward(Constants.LIFT_MAX_POWER);
        // liftMotorLeft.configPeakOutputReverse(Constants.LIFT_MAX_POWER);

        // Configure left motor as a follower to right motor
        //liftMotorLeft.follow(liftMotorRight);
        //liftMotorLeft.setInverted(InvertType.OpposeMaster);
        liftMotorLeft.setInverted(true);

        // Put a hard limit on motor power to limit potential damage
        // liftMotorRight.configNominalOutputForward(0);
        // liftMotorRight.configNominalOutputReverse(0);
        // liftMotorRight.configPeakOutputForward(Constants.LIFT_MAX_POWER);
        // liftMotorRight.configPeakOutputReverse(Constants.LIFT_MAX_POWER);

        // Ensure the motors are stopped
        liftMotorRight.set(ControlMode.PercentOutput, 0.0);   // Clear any outstanding Motion Magic commands and park the motor
        liftMotorLeft.set(ControlMode.PercentOutput, 0.0);

        // Configure right motor for motion magic
        liftMotorRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        liftMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        liftMotorRight.setSelectedSensorPosition(0);
        liftMotorLeft.setSelectedSensorPosition(0);
        liftMotorRight.configNeutralDeadband(Constants.LIFT_DEADBAND);  // The deadband should be very small to allow precise control
        liftMotorLeft.configNeutralDeadband(Constants.LIFT_DEADBAND);  // The deadband should be very small to allow precise control


         // PID values for raising arm
        liftMotorRight.config_kP(RAISE_PID_SLOT, Constants.LIFT_UP_P);
        liftMotorRight.config_kI(RAISE_PID_SLOT, Constants.LIFT_UP_I);
        liftMotorRight.config_kD(RAISE_PID_SLOT, Constants.LIFT_UP_D);
        liftMotorRight.config_kF(RAISE_PID_SLOT, Constants.LIFT_UP_F);

        // PID values for lowering LIFT
        liftMotorRight.config_kP(LOWER_PID_SLOT, Constants.LIFT_DOWN_P);
        liftMotorRight.config_kI(LOWER_PID_SLOT, Constants.LIFT_DOWN_I);
        liftMotorRight.config_kD(LOWER_PID_SLOT, Constants.LIFT_DOWN_D);
        liftMotorRight.config_kF(LOWER_PID_SLOT, Constants.LIFT_DOWN_F);

        liftMotorRight.selectProfileSlot(RAISE_PID_SLOT, 0);
  
        // Motion Magic parameters
        liftMotorRight.configMotionSCurveStrength(Constants.LIFT_MM_SMOOTHING);
        liftMotorRight.configMotionCruiseVelocity(Constants.LIFT_MM_CRUISE_VELO);
        liftMotorRight.configMotionAcceleration(Constants.LIFT_MM_ACCEL);
    }

    public void reset() {
        // Assume lift is retracted and reset encoders
        liftMotorRight.setSelectedSensorPosition(0);
        liftMotorLeft.setSelectedSensorPosition(0);

        // Simultaneously, flag the lift as *not* being down all the way
        rightArmParked = false;
        leftArmParked  = false;

        // Stop any motor activity
        liftMotorRight.set(ControlMode.PercentOutput, 0.0);
        liftMotorLeft.set(ControlMode.PercentOutput, 0.0);

        // Have left motor follow right motor
        // Configure left motor as a follower to right motor
        liftMotorLeft.follow(liftMotorRight);
        liftMotorLeft.setInverted(InvertType.OpposeMaster);
    }


    /**
     * Control the lift
     * 
     */
    public void moveLift(double liftPower) {
        double positionRight = liftMotorRight.getSelectedSensorPosition();
        double positionLeft = liftMotorLeft.getSelectedSensorPosition();


        if (((liftPower < 0) && (positionRight > Constants.LIFT_RIGHT_MIN_POSITION)) ||
            ((liftPower > 0) && (positionRight < 0)))
            liftMotorRight.set(ControlMode.PercentOutput, liftPower);
        else
            liftMotorRight.set(ControlMode.PercentOutput, 0);

        if (((liftPower < 0) && (positionLeft > Constants.LIFT_LEFT_MIN_POSITION)) ||
            ((liftPower > 0) && (positionLeft < 0)))
            liftMotorLeft.set(ControlMode.PercentOutput, liftPower);
        else
            liftMotorLeft.set(ControlMode.PercentOutput, 0);
        
        SmartDashboard.putNumber("Right Pos", liftMotorRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Pos", liftMotorLeft.getSelectedSensorPosition());
    }


    // public void liftPeriodic(double liftPower) {
    //     double climberPosition   = liftMotorRight.getSelectedSensorPosition();
    //     double nextClimbPosition = climberPosition + (liftPower * Constants.LIFT_CHANGE_POSITION);

    //     if (nextClimbPosition > Constants.LIFT_MAX_POSITION){
    //         nextClimbPosition = Constants.LIFT_MAX_POSITION;
    //     }
    //     if (nextClimbPosition < Constants.LIFT_MIN_POSITION){
    //         nextClimbPosition = Constants.LIFT_MIN_POSITION;
    //     }

    //     liftMotorRight.set(ControlMode.MotionMagic, nextClimbPosition, DemandType.ArbitraryFeedForward, Constants.LIFT_FEED_FORWARD);
    // }
    

    /**
     * Put arms into the parked position
     */
    public void pullArmDown() {
        liftMotorLeft.setInverted(true);
        // Pull the right and left arms down into the parked position
        if (!rightArmParked)
            liftMotorRight.set(ControlMode.PercentOutput, Constants.LIFT_PARK_POWER);

        if (!leftArmParked)
            liftMotorLeft.set(ControlMode.PercentOutput, Constants.LIFT_PARK_POWER);
            
        //
        // If motor current peaks, it should be stalled at the bottom
        // Stop motor and reset encoder to 0
        //
        if (Math.abs(liftMotorRight.getStatorCurrent()) > Constants.LIFT_PARKED_CURRENT) {
            rightArmParked = true;
            liftMotorRight.set(ControlMode.PercentOutput, 0.0);
            liftMotorRight.setSelectedSensorPosition(0);
        }

        if (Math.abs(liftMotorLeft.getStatorCurrent()) > Constants.LIFT_PARKED_CURRENT) {
            leftArmParked = true;
            liftMotorLeft.set(ControlMode.PercentOutput, 0.0);
            liftMotorLeft.setSelectedSensorPosition(0);
        }

        SmartDashboard.putBoolean("Right Parked", rightArmParked);
        SmartDashboard.putBoolean("Left Parked",  leftArmParked);
    }
}