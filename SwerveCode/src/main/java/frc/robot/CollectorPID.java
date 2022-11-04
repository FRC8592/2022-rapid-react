package frc.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.DigitalInput;


public class CollectorPID {
  private double kP;
  private double kI;
  private double kD;
  private double collectorSpeed;
  private WPI_TalonFX armMotor;

  public CollectorPID(double kP, double kI, double kD){
    this.kP=kP;
    this.kI=kI;
    this.kD=kD;
    collectorSpeed = 0;

    //initialize and configure motors
    armMotor = new WPI_TalonFX(Constants.newFlywheelCollector);
    armMotor.configFactoryDefault();
    armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    


    //set PID values
    armMotor.config_kP(0, kP);
    armMotor.config_kI(0, kI);
    armMotor.config_kD(0, kD);

    //make sure motor is at 0 speed
    armMotor.set(ControlMode.PercentOutput, collectorSpeed);




  }  

  public void updateVelocity(double targetSpeed, int mode){
    double targetSpeedTicks = Constants.RPM_TO_TICKS_MS * targetSpeed;
    
    
    armMotor.set(ControlMode.Velocity, targetSpeedTicks);


  }

  public double getVelocity(double currentSpeedTicks){
    double currentSpeedRPM = armMotor.getSelectedSensorVelocity() / Constants.RPM_TO_TICKS_MS;

    return currentSpeedRPM;
  }

}