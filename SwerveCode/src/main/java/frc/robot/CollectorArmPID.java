//////////////////////////////////////////////////////////////////////////////////////////////////////
// Manage the collector arm with Motion Magic
/////////////////////////////////////////////////////////////////////////////////////////////////////
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.ColorSensor.BALL_COLOR;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;


public class CollectorArmPID {
    // Configuration constants
    private static final double ARM_RAISE_POWER       = 0.3;   // Power for raising arm
    private static final double ARM_LOWER_POWER       = 0.3;   // Power for lowering arm
    private static final double ARM_COLLECT_POWER     = 0.1;   // Power for pushing down on balls
    private static final int    ARM_LOW_POS_THRESHOLD = 1000;  // TODO determine this value empirically
    private static final int    RAISE_PID_SLOT = 0;
    private static final int    LOWER_PID_SLOT = 1;

    // State values
    private static enum armStates {ARM_UP, ARM_RAISING, ARM_DESCENDING, ARM_COLLECTING}

    // Object variables
    private WPI_TalonFX  armMotor;
    private DigitalInput limitSwitch;

    // Internal global variables
    private armStates armState = armStates.ARM_UP;  // Robot should start with arm in up position


    // Configure the arm motor and limit switch
    public CollectorArmPID () {
        // Create the arm motor object
        armMotor = new WPI_TalonFX(Constants.COLLECTOR_ARM_CAN);
        armMotor.setInverted(true);
        armMotor.setNeutralMode(NeutralMode.Brake);

        // Select and reset the encoder for the arm
        armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        armMotor.setSelectedSensorPosition(0);

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
        //armMotor.configClosedloopRamp(0.5, 0);

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
     * Control the arm state machine
     */
    public void update(){
        SmartDashboard.putBoolean("limit switch value", limitSwitch.get());
        SmartDashboard.putString("Arm State", armState.toString());
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
                
                armMotor.selectProfileSlot(RAISE_PID_SLOT, 0);
                armMotor.set(ControlMode.Position, 0);

                // // Slow the arm as it approaches the limit switch
                // else if(armMotor.getSelectedSensorPosition() > -500){
                //     armMotor.set(0.1);

                // }

                // // Apply normal power to raise the arm
                // else {
                //     armMotor.set(0.25);
                // }
                
                break;

            case ARM_DESCENDING:
                // Apply power to lower arm.  Power is proportional to angle.
                armMotor.selectProfileSlot(LOWER_PID_SLOT, 0);
                armMotor.set(ControlMode.Position, Constants.BALL_SET_POINT);

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