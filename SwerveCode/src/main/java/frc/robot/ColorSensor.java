//////////////////////////////////////////////////////////////////////////////////////////////////////
// Identify the color of an ingested ball at start up (alliance color) and during collection
/////////////////////////////////////////////////////////////////////////////////////////////////////

//code by Zolton
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;


public class ColorSensor{
    // Color sensor I2C interface
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    // Alliance color enumeration
    public static enum BALL_COLOR {RED, BLUE, NONE} // DO NOT CHANGE ORDER.  Limelight configuration depends on red, blue, none order
    
    // Internal ball color tracking
    private BALL_COLOR allianceColor    = BALL_COLOR.NONE;
    private BALL_COLOR currentBallColor = BALL_COLOR.NONE;


    /**
     * Set the alliance color for the match based on the ball loaded into the robot
     */
    public ColorSensor() {
        int i = 0;
        
        //
        // Loop until we get a real color (e.g. RED or BLUE) or until we exhaust our maximum tries
        //
        // If we never get a real color, pick BLUE.  We have a 50% chance of being right for autonmous, and the 
        // co-driver can override to RED, if necessary, for teleop.
        //
        while ((allianceColor == BALL_COLOR.NONE) && (i < Constants.MAX_COLOR_CHECKS)) {
            allianceColor = updateCurrentBallColor();      // Get the color and assign it to the alliance color
            i++;
        }

        // Force BLUE if the sensor failed to detect a color
        if (allianceColor == BALL_COLOR.NONE)
            allianceColor = BALL_COLOR.BLUE;

        SmartDashboard.putString("Alliance", allianceColor.toString());
    }


    /**
     * In case we can't start with a ball near the sensor, allow drivers to force blue alliance color
     */
    public void forceBlueAlliance() {
        allianceColor = BALL_COLOR.BLUE;
    }


    /**
     * In case we can't start with a ball near the sensor, allow drivers to force red alliance color
     */
    public void forceRedAlliance() {
        allianceColor = BALL_COLOR.RED;
    }


    /**
     * 
     * @return Alliance color encoded as BALL_COLOR
     */
    public BALL_COLOR getAllianceColor() {
        return allianceColor;
    }


     /**
     * 
     * @return Ball color encoded as BALL_COLOR
     */
    public BALL_COLOR getBallColor() {
        return currentBallColor;
    }


    /**
     * 
     * @return Proximity reading from the color sensor
     */
    public int getProximity(){
        return m_colorSensor.getProximity();
    }


    /**
     * 
     * @return true if the ball in the upper position matches the alliance ball color
     */
    public boolean isAllianceBallColor() {
        if ((currentBallColor == allianceColor) || (currentBallColor == BALL_COLOR.NONE))
            return true;
        else
            return false;
    }


    /**
     * Read the color of the lower ball from the color sensor
     * 
     * @return Ball color encoded as BALL_COLOR
     */
    public BALL_COLOR updateCurrentBallColor(){

        // Get raw color data from the color sensor
        Color detectedColor = m_colorSensor.getColor();
        //
        // If we have a ball in position, select between a RED and BLUE ball
        // If the color sensor values are indeterminate, force a BLUE result
        // if no ball is in position, we return NONE
        //
        if (m_colorSensor.getProximity() >= Constants.MIN_BALL_PROXIMITY)   // Make sure a ball is in position
            if (detectedColor.blue >= detectedColor.red)
                currentBallColor =  BALL_COLOR.BLUE;
            else
                currentBallColor = BALL_COLOR.RED;
        else
            currentBallColor = BALL_COLOR.NONE;

        if (currentBallColor != BALL_COLOR.NONE && this.allianceColor == this.allianceColor.NONE)  {
            this.allianceColor = currentBallColor;
        }                        // Only return NONE if a ball is not in position
                                // Place raw color data on the dashboard for diagnostics
        SmartDashboard.putNumber("Red", detectedColor.red);                     // Red component of ball color
        SmartDashboard.putNumber("Blue", detectedColor.blue);                   // Blue component of ball color
        SmartDashboard.putNumber("Proximity", m_colorSensor.getProximity());    // Distance to ball (bigger numbers are closer). ~200 when no ball present
        SmartDashboard.putString("Color", currentBallColor.toString());          // Computed ball color

        return currentBallColor;
    }

}