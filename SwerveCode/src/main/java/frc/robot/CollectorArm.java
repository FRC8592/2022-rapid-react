//////////////////////////////////////////////////////////////////////////////////////////////////////
// Control the up & down positioning of the collector arm
/////////////////////////////////////////////////////////////////////////////////////////////////////

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;


public class CollectorArm {
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

    //
    // Configure the arm motor
    //
    public CollectorArm () {
        // Create the arm motor object
        armMotor = new WPI_TalonFX(Constants.ARM_MOTOR_CAN);

        // Select and reset the encoder for the arm
        armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 ,0);
        armMotor.setSelectedSensorPosition(0);
    }

    //
    // Control the arm motion
    //

    

    //
    // Move arm up
    //


    //
    // Move arm down
    //
    
    
}
