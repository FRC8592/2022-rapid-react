//////////////////////////////////////////////////////////////////////////////////////////////////////
// Manage the collector arm with Motion Magic
/////////////////////////////////////////////////////////////////////////////////////////////////////
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;


public class Climber {
    // Configuration constants
    private static final int    RAISE_PID_SLOT = 0;
    private static final int    LOWER_PID_SLOT = 1;

    // Object variables
    private WPI_TalonFX liftMotorRight;
    private WPI_TalonFX liftMotorLeft;

    // State values
    private static enum liftStates {LIFT_UP, LIFT_RAISING, LIFT_DESCENDING, LIFT_DOWN}
    
    // Internal global variables
    private liftStates liftState;


    // Configure the arm motor and limit switch
    public Climber () {

        // Robot should start with the lift in the down position
        liftState = liftStates.LIFT_DOWN;

        // Create the lift motor objects and clear configuration to factory defaults
        liftMotorRight = new WPI_TalonFX(Constants.LIFT_RIGHT_CAN);
        liftMotorLeft  = new WPI_TalonFX(Constants.LIFT_LEFT_CAN);
        liftMotorRight.configFactoryDefault();
        liftMotorLeft.configFactoryDefault();

        // Configure voltage compensation and current limiting to both motors
        liftMotorRight.configVoltageCompSaturation(Constants.LIFT_VOLTAGE);
        liftMotorLeft.configVoltageCompSaturation(Constants.LIFT_VOLTAGE);
        liftMotorRight.enableVoltageCompensation(true);   // Enable voltage compensation
        liftMotorLeft.enableVoltageCompensation(true);    // Enable voltage compensation
        liftMotorRight.configSupplyCurrentLimit(Constants.LIFT_CURRENT_LIMIT);
        liftMotorLeft.configSupplyCurrentLimit(Constants.LIFT_CURRENT_LIMIT);

        // Set motors to brake mode when idle
        liftMotorLeft.setNeutralMode(NeutralMode.Brake);
        liftMotorRight.setNeutralMode(NeutralMode.Brake);

        // Configure left motor as a follower to right motor
        liftMotorLeft.follow(liftMotorRight);
        liftMotorLeft.setInverted(InvertType.OpposeMaster);

        // Ensure the motors are stopped
        liftMotorRight.set(ControlMode.PercentOutput, 0.0);   // Clear any outstanding Motion Magic commands and park the motor

        // Configure right motor for motion magic
        liftMotorRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        liftMotorRight.setSelectedSensorPosition(0);        // THIS COULD BE A DANGEROUS ASSUMPTION ABOUT OUR STARTING POINT!!!
        liftMotorRight.configNeutralDeadband(Constants.LIFT_DEADBAND);

        // Set current limits so we don't blow any fuses
        liftMotorRight.configSupplyCurrentLimit(Constants.LIFT_CURRENT_LIMIT);

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
  
        // Motion Magic parameters
        liftMotorRight.configMotionSCurveStrength(Constants.LIFT_MM_SMOOTHING);
        liftMotorRight.configMotionCruiseVelocity(Constants.LIFT_MM_CRUISE_VELO);
        liftMotorRight.configMotionAcceleration(Constants.LIFT_MM_ACCEL);
    }


    /**
     * Control the lift
     */
    public void moveLift(double liftPower) {
        liftMotorRight.set(ControlMode.PercentOutput, liftPower);

    }
    
}