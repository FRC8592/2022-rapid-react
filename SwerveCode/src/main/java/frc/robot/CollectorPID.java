package frc.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class CollectorPID {
  private double kP;
  private double kI;
  private double kD;
  private double kF;
  private double collectorSpeed;
  private WPI_TalonFX armMotor;

  public CollectorPID(double kP, double kI, double kD, double kF){
    this.kP=kP;
    this.kI=kI;
    this.kD=kD;
    this.kF=kF;
    collectorSpeed = 0;

    //initialize and configure motors
    armMotor = new WPI_TalonFX(Constants.newFlywheelCollector);
    armMotor.configFactoryDefault();
    armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    armMotor.setInverted(true);
    armMotor.setNeutralMode(NeutralMode.Brake);
    


    //set PID values
    armMotor.config_kP(0, kP);
    armMotor.config_kI(0, kI);
    armMotor.config_kD(0, kD);
    armMotor.config_kF(0, kF);
    armMotor.selectProfileSlot(0, 0);
    //make sure motor is at 0 speed
    armMotor.set(ControlMode.Velocity, collectorSpeed);
  }  

  public void updateVelocity(double targetSpeed, int mode){
    double targetSpeedTicks = Constants.RPM_TO_TICKS_MS * targetSpeed;
    armMotor.set(ControlMode.Velocity, targetSpeedTicks);

    SmartDashboard.putNumber("Collector Rotation Expected Speed", targetSpeed);
    //SmartDashboard.putNumber("Collector Rotation Actual Speed", this.getVelocity());
    SmartDashboard.putNumber("Collector Rotation Error", this.getVelocity()-targetSpeed);
  }

  public double getVelocity(){
    double currentSpeedRPM = armMotor.getSelectedSensorVelocity() / Constants.RPM_TO_TICKS_MS;

    return currentSpeedRPM;
  }

}