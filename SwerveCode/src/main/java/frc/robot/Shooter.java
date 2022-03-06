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

    //constants
    Collector collector;

    //motor controllors
    public WPI_TalonFX processing;
    public WPI_TalonFX flyWheelLeft;
    public WPI_TalonFX flyWheelRight;
    public WPI_TalonFX staging;

    public enum ShooterState{AUTONOMOUS,SHOOT}
    private ShooterState shooterState;

    double flywheelVelocity;


    public Shooter(){
        collector     = new Collector();
        processing    = new WPI_TalonFX(Constants.newFlywheelCollector);
        flyWheelLeft  = new WPI_TalonFX(Constants.newFlywheelLeft);
        flyWheelRight = new WPI_TalonFX(Constants.newFlywheelRight);
        staging       = new WPI_TalonFX(Constants.newFlywheelStaging);

        flyWheelLeft.follow(flyWheelRight);
        flyWheelLeft.setInverted(InvertType.OpposeMaster);

        flyWheelRight.setInverted(false);
        processing.setInverted(true);
        staging.setInverted(true);

        flywheelVelocity = SmartDashboard.getNumber("enter velocity", 10);
        SmartDashboard.putNumber("Flywheel Velocity", flywheelVelocity);

        flywheelVelocity = -0.5;
        flyWheelRight.set(ControlMode.PercentOutput, flywheelVelocity);
    }

    public ShooterState determineShooterState(){
        CollectorState collectorState = collector.determineCollectorState();

        if(collectorState != CollectorState.NO_BALLS_LOADED){
            shooterState = ShooterState.SHOOT;
        }else{
            shooterState = ShooterState.AUTONOMOUS;
        }

        SmartDashboard.putString("State", shooterState.toString());
        return shooterState;
    }

    public void collectorDriverControl(XboxController shootController){

        switch (determineShooterState()) {
            case SHOOT:
                if(shootController.getBButton()){
                    manualControl();
                    
                }else{
                    collector.ballControl();
                }
                break;

            case AUTONOMOUS:
                collector.ballControl();
                break;
        
        }
    }


    public void manualControl(){
        collector.driveStagingWheels(1);
        collector.driveProcessingWheels(1);
        SmartDashboard.putNumber("Running Manual", 1);
    }
}
    


    

