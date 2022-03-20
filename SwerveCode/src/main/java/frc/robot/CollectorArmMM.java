//////////////////////////////////////////////////////////////////////////////////////////////////////
// Manage the collector arm with Motion Magic
/////////////////////////////////////////////////////////////////////////////////////////////////////
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.DigitalInput;


public class CollectorArmMM {
    // Configuration constants
    private static final int    RAISE_PID_SLOT = 0;
    private static final int    LOWER_PID_SLOT = 1;

    // State values
    private static enum armStates {ARM_UP, ARM_RAISING, ARM_DESCENDING, ARM_COLLECTING}

    // Object variables
    private WPI_TalonFX  armMotor;
    private DigitalInput limitSwitch;

    // Internal global variables
    private armStates armState;


    // Configure the arm motor and limit switch
    public CollectorArmMM () {

        // Robot should start with arm in up position
        armState = armStates.ARM_UP;  

        // Create the arm motor object
        armMotor = new WPI_TalonFX(Constants.COLLECTOR_ARM_CAN);
        armMotor.configFactoryDefault();
        armMotor.setInverted(true);
        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.set(ControlMode.PercentOutput, 0.0);   // Clear any outstanding Motion Magic commands and park the motor

        // Select and reset the encoder for the arm
        armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        armMotor.setSelectedSensorPosition(0);
        armMotor.configNeutralDeadband(Constants.ARM_DEADBAND);

        // Set current limits so we don't burn up if we get stalled
        //
        armMotor.configSupplyCurrentLimit(Constants.ARM_CURRENT_LIMIT);

        // PID values for raising arm
        armMotor.config_kP(RAISE_PID_SLOT, Constants.ARM_UP_P);
        armMotor.config_kI(RAISE_PID_SLOT, Constants.ARM_UP_I);
        armMotor.config_kD(RAISE_PID_SLOT, Constants.ARM_UP_D);
        armMotor.config_kF(RAISE_PID_SLOT, Constants.ARM_UP_F);

        // PID values for lowering arm
        armMotor.config_kP(LOWER_PID_SLOT, Constants.ARM_DOWN_P);
        armMotor.config_kI(LOWER_PID_SLOT, Constants.ARM_DOWN_I);
        armMotor.config_kD(LOWER_PID_SLOT, Constants.ARM_DOWN_D);
        armMotor.config_kF(LOWER_PID_SLOT, Constants.ARM_DOWN_F);
  
        // Motion Magic parameters
        armMotor.configMotionSCurveStrength(Constants.MM_SMOOTHING);
        armMotor.configMotionCruiseVelocity(Constants.MM_CRUISE_VELO);
        armMotor.configMotionAcceleration(Constants.MM_ACCEL);

        // Instantiate the limit switch
        limitSwitch = new DigitalInput(Constants.COLLECTOR_ARM_LIMIT_SWITCH);
    }


    /**
     * Raise the collector arm to the parked position
     */
    public void raiseArm() {
        // Only start raising the arm if we are descending or already in the down position
        if ((armState == armStates.ARM_DESCENDING) || (armState == armStates.ARM_COLLECTING))
            armState = armStates.ARM_RAISING;
    }

    
    /**
     * Lower the collector arm to the collecting position
     */
    public void lowerArm() {
        // Only start lowering the arm if we are raising or already in the up position
        if ((armState == armStates.ARM_RAISING) || (armState == armStates.ARM_UP))
            armState = armStates.ARM_DESCENDING;
    }


    /**
     * 
     * @param position The position of the arm, in encoder ticks
     * @return Feed Forward power (-1, 1)
     */
    private double calcFeedForward(double position) {
        double angleRad = (-position / Constants.ARM_TICKS_180) * 3.1415;

        return Constants.ARM_STEADY_POWER * Math.sin(angleRad);
    }


    /**
     * Control the arm state machine
     */
    public void update(){
        double feedForward = calcFeedForward(armMotor.getSelectedSensorPosition());
    
        SmartDashboard.putBoolean("limit switch value", limitSwitch.get());
        SmartDashboard.putString("Arm State", armState.toString());
        SmartDashboard.putNumber("Feed Forward", feedForward);
        SmartDashboard.putNumber("Collector arm position", armMotor.getSelectedSensorPosition());
        
        switch (armState) {

            case ARM_UP:
                // Disable power if the arm is pressed against the switch
                if(limitSwitch.get() == false){
                    armMotor.set(ControlMode.PercentOutput, 0.0);
                    armMotor.setSelectedSensorPosition(0);
                }

                // If the arm moves away from the switch, go back to ARM_RAISING to recover
                else{
                    armState = armStates.ARM_RAISING;
                }
                break;

            case ARM_RAISING:
                // Stop when the limit switch is hit
                if (limitSwitch.get() == false){
                    armState = armStates.ARM_UP;

                }
                
                // Raise arm to position 0
                armMotor.selectProfileSlot(RAISE_PID_SLOT, 0);
                armMotor.set(ControlMode.MotionMagic, 100, DemandType.ArbitraryFeedForward, feedForward);

               
                break;

            case ARM_DESCENDING:
                // Apply power to lower arm.  Power is proportional to angle.
                armMotor.selectProfileSlot(LOWER_PID_SLOT, 0);
                armMotor.set(ControlMode.MotionMagic, Constants.BALL_SET_POINT, DemandType.ArbitraryFeedForward, feedForward);

                if (armMotor.getSelectedSensorPosition() <= Constants.BALL_SET_POINT)
                    armState = armStates.ARM_COLLECTING;

                break;

            case ARM_COLLECTING:
                //
                // If we are below BALL_SET_POINT.  If we are above, apply a bit more power
                // to push down on the ball we are probably collecting
                //
                if (armMotor.getSelectedSensorPosition() < Constants.BALL_SET_POINT)
                    armMotor.set(ControlMode.PercentOutput, 0.0);
                else
                    armMotor.set(ControlMode.PercentOutput, -0.10);

                break;

        }

    }
    
}