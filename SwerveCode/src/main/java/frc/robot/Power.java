//////////////////////////////////////////////////////////////////////////////////////////////////////
// Power measurement and control
/////////////////////////////////////////////////////////////////////////////////////////////////////

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Power {
    // Object variables
    private PowerDistribution revPDH;

    //
    // Shared power variables
    //
    public double temp;
    public double voltage;
    public double current;
    public double power;
    public double energy;

    //
    // Constructor for power control
    //
    public Power() {
        // Create new Rev Power Distribution object
        revPDH = new PowerDistribution();
    }


    //
    // Periodically post power data to the dashboard
    //
    public void powerPeriodic() {
        // Get parameters from the PDH\
        temp    = revPDH.getTemperature();
        voltage = revPDH.getVoltage();
        current = revPDH.getTotalCurrent();
        power   = revPDH.getTotalPower();
        energy  = revPDH.getTotalEnergy();

        // Place all parameters onto Smart Dashboard
        SmartDashboard.putNumber("Power/Temperature", temp);
        SmartDashboard.putNumber("Power/Voltage", voltage);
        SmartDashboard.putNumber("Power/Current", current);
        SmartDashboard.putNumber("Power/Power", power);
        SmartDashboard.putNumber("Power/Energy", energy);
    }


    //
    // Turn the switchable 12v port on
    //
    public void relayOn() {
        revPDH.setSwitchableChannel(true);
    }


    //
    // Turn the switchable 12v port off
    //
    public void relayOff() {
        revPDH.setSwitchableChannel(false);
    }

}