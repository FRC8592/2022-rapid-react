package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController; //this puts in the xbox contoller stuff
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class shooter {

    //constants

    //motor controllors
    public WPI_TalonFX collector;
    public WPI_TalonFX flyWheelLeft;
    public WPI_TalonFX flyWheelRight;
    public WPI_TalonFX stagingMotor;
    
    //motor groups
    SpeedControllerGroup flyWheel;

    public void testshooter(XboxController shooterController) {

        WPI_TalonFX collector = new WPI_TalonFX(config_hw.newFlywheelCollector);
        WPI_TalonFX flyWheelLeft = new WPI_TalonFX(config_hw.newFlywheelLeft);
        WPI_TalonFX flyWheelRight = new WPI_TalonFX(config_hw.newFlywheelRight);
        WPI_TalonFX staging = new WPI_TalonFX(config_hw.newFlywheelStaging);

            double flyWheelSpeed;
            flyWheelRight.setInverted(true);
    
            //link flywheel motors
            flyWheel  = new SpeedControllerGroup(flyWheelLeft, flyWheelRight);
    
            //control speed of the flywheel
            flyWheelSpeed = shooterController.getX(GenericHID.Hand.kRight);
            collector.set(shooterController.getTriggerAxis(GenericHID.Hand.kLeft));
            staging.set(shooterController.getTriggerAxis(GenericHID.Hand.kRight));
    
                flyWheel.set(flyWheelSpeed);
            }

    }
    

