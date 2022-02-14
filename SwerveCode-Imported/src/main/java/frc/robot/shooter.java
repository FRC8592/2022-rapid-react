package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController; //this puts in the xbox contoller stuff
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class shooter{ 

    //constants
    private static final int LineBreakSensorPort = 0;

    //motor controllors
    public WPI_TalonFX collector;
    public WPI_TalonFX flyWheelLeft;
    public WPI_TalonFX flyWheelRight;
    public WPI_TalonFX staging;
    
    //motor groups
    MotorControllerGroup flyWheel;

    // Line break sensor
    private DigitalInput lineSensorA;

    public shooter(){
        collector = new WPI_TalonFX(config_hw.newFlywheelCollector);
        flyWheelLeft = new WPI_TalonFX(config_hw.newFlywheelLeft);
        flyWheelRight = new WPI_TalonFX(config_hw.newFlywheelRight);
        staging = new WPI_TalonFX(config_hw.newFlywheelStaging);
        flyWheel  = new MotorControllerGroup(flyWheelLeft, flyWheelRight);

        lineSensorA = new DigitalInput(LineBreakSensorPort);

        flyWheelLeft.setInverted(true);
        collector.setInverted(true);
    }

    public void testshooter(XboxController shooterController) {

        // Line break sensor

        SmartDashboard.putBoolean("Line Sensor", lineSensorA.get());

        double flyWheelSpeed;

        //link flywheel motors

        //control speed of the flywheel
        flyWheelSpeed = shooterController.getRightTriggerAxis();
        collector.set(shooterController.getLeftTriggerAxis());
        staging.set(shooterController.getLeftTriggerAxis());
    
            flyWheel.set(flyWheelSpeed);
        }
    }
    

