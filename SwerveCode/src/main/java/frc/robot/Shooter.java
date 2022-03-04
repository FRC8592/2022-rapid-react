package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Collector.CollectorState;
import edu.wpi.first.wpilibj.XboxController; //this puts in the xbox contoller stuff
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import java.util.stream.Collectors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Shooter{ 

    //constants\
    Collector collector;

    //motor controllors
    public WPI_TalonFX processing;
    public WPI_TalonFX flyWheelLeft;
    public WPI_TalonFX flyWheelRight;
    public WPI_TalonFX staging;
    
    //motor groups
    MotorControllerGroup flyWheel;

    public enum ShooterState{TWO_BALL_MANUAL,AUTONOMOUS,MANUAL_OVERRIDE}
    private ShooterState shooterState;

    double flywheelVelocity;
    public Shooter(){
        processing    = new WPI_TalonFX(Constants.newFlywheelCollector);
        flyWheelLeft  = new WPI_TalonFX(Constants.newFlywheelLeft);
        flyWheelRight = new WPI_TalonFX(Constants.newFlywheelRight);
        staging       = new WPI_TalonFX(Constants.newFlywheelStaging);

        flyWheelLeft.follow(flyWheelRight);

        flyWheelRight.setInverted(false);
        flyWheelLeft.setInverted(InvertType.OpposeMaster);
        processing.setInverted(true);

        flywheelVelocity = SmartDashboard.getNumber("enter velocity", 0);
        SmartDashboard.putNumber("Flywheel Velocity", flywheelVelocity);
    }

    public ShooterState determineShooterState(XboxController shootController){
        CollectorState collectorState = collector.determineCollectorState();

        if(shootController.getBButtonPressed()){
            shooterState = ShooterState.MANUAL_OVERRIDE;
        }else if(collectorState == CollectorState.TWO_BALLS){
            shooterState = ShooterState.TWO_BALL_MANUAL;
        }else{
            shooterState = ShooterState.AUTONOMOUS;
        }

        return shooterState;
    }

    public void collectorDriverControl(XboxController shootController){

        switch (determineShooterState(shootController)) {
            case MANUAL_OVERRIDE:
                this.manualControl(shootController);
                break;
            
            case TWO_BALL_MANUAL:
                this.manualControl(shootController);
                break;

            case AUTONOMOUS:
                collector.ballControl();
                break;
        
        }
    }


    public void manualControl(XboxController shootController){

        if(shootController.getXButton()){
            flyWheelRight.set(ControlMode.PercentOutput, shootController.getLeftTriggerAxis());
        } else {
            flyWheelRight.set(ControlMode.Velocity, flywheelVelocity);
        }   

        processing.set(shootController.getRightTriggerAxis());
        staging.set(shootController.getRightTriggerAxis());
    }
}
    


    

