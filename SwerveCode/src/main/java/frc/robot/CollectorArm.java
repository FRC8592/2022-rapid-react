package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;


public class CollectorArm {

    DigitalInput limitSwitch;
    // Configuration constants
    private static final double ARM_RAISE_POWER       = 0.3;   // Power for raising arm
    private static final double ARM_LOWER_POWER       = 0.3;   // Power for lowering arm
    private static final double ARM_COLLECT_POWER     = 0.1;   // Power for pushing down on balls
    private static final int    ARM_LOW_POS_THRESHOLD = 1000;  // TODO determine this value empirically
    // State values
    private static enum armStates {ARM_UP, ARM_RAISING, ARM_DESCENDING, ARM_COLLECTING}

    // Object variables
    private WPI_TalonFX armMotor;

    // Internal global variables
    private armStates armState = armStates.ARM_UP;  // Robot should start with arm in up position


    // Configure the arm motor
    public CollectorArm () {
        // Create the arm motor object
        armMotor = new WPI_TalonFX(Constants.intakePosition);

        // Select and reset the encoder for the arm
        armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        armMotor.setSelectedSensorPosition(0);

        //PID values
        armMotor.config_kP(0, 0);
        armMotor.config_kI(0, 0);
        armMotor.config_kD(0, 0);
        armMotor.config_kF(0, 0);
        armMotor.configClosedloopRamp(0.5, 0);

        limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_TOP_PORT);

    }

    public void update(){
        SmartDashboard.putBoolean("limit switch value", limitSwitch.get());

        switch (armState) {
            case ARM_UP:
                if(limitSwitch.get())
                break;
            case ARM_RAISING:

                break;
            case ARM_DESCENDING:

                break;
            case ARM_COLLECTING:

                break;

        }

    }

    // Move arm up
    public void startIntakeUp(){


    }

    // Move arm down
    public void startIntakeDown(){

        
    }
    
}