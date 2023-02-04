package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private TalonSRX rollerMotor;
    // private final BeamBreak beamBreak;

    SparkMaxPIDController motorPID;

    public Intake() {
        // rollerMotor = new CANSparkMax(62, MotorType.kBrushed);
        rollerMotor = new TalonSRX(62);
        //PID constants WIP
        // motorPID = rollerMotor.getPIDController();
        // motorPID.setP(0.01);
        // motorPID.setI(0);
        // motorPID.setD(0);
        // beamBreak = new BeamBreak(1, 0.1);
    }

    // Active rollers and stop once an item is collected
    public void intakePiece(double speed) {
        // if(!beamBreak.isBroken()) {
            // motorPID.setReference(0.25, ControlType.kVelocity);
            // rollerMotor.set(Math.abs(speed));
            rollerMotor.set(TalonSRXControlMode.PercentOutput, speed);
        // }
    }

    // Reverse rollers to intake cube
    public void outtakePiece(double speed) {
        // motorPID.setReference(-0.25, ControlType.kVelocity);
        // rollerMotor.set(-Math.abs(speed));
            rollerMotor.set(TalonSRXControlMode.PercentOutput, -speed);
    }

    public void stop() {
        rollerMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }
    
    public void BeamBroken() {
        // SmartDashboard.putBoolean("BB is broken", beamBreak.isBroken());
        // SmartDashboard.putBoolean("DI Beam Broken?", !beamBreak.get());
    }
}
