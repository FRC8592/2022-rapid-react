//code by Zolton
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance; // idk what this is but im keeping it
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.ALLIANCE_COLOR;

import com.revrobotics.ColorSensorV3;

public class ColorSensor{
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    

    private ALLIANCE_COLOR alliance;
    private ALLIANCE_COLOR currentBallColor;


    public ColorSensor(){
        alliance = updateCurrentBallColor();
        currentBallColor = alliance;
    }

    public void getColors(){
        Color detectedColor = m_colorSensor.getColor();
        int proximity = m_colorSensor.getProximity();
      
        SmartDashboard.putNumber("Red", detectedColor.red); //output the seen overall red value to the smartdashboard
        SmartDashboard.putNumber("Blue", detectedColor.blue); //output the seen overall blue value to the smartdashboard
        SmartDashboard.putNumber("Proximity", proximity);  //distance from colorsensor to the nearest object (2047 to 0) output to the smartdashboard
        SmartDashboard.putString("alliance", this.toString()); //current alliance value output the the smartdashboard
        //System.out.println(detectedColor.red);
    }

    public ALLIANCE_COLOR updateCurrentBallColor(){
        Color detectedColor = m_colorSensor.getColor();
        if(detectedColor.blue > detectedColor.red){
          return ALLIANCE_COLOR.BLUE;
        }else if(detectedColor.red > detectedColor.blue){
          return ALLIANCE_COLOR.RED;
        }else{
            return ALLIANCE_COLOR.NONE;}
      }

    public boolean compareBallToAlliance(){
        if(currentBallColor == alliance){
            return true;
        }else if(currentBallColor != alliance){
            return false;
        }else{return false;}
    }

    public int getProximity(){
        //int proximity = m_colorSensor.getProximity();
        return m_colorSensor.getProximity();
    }

    public ALLIANCE_COLOR getAlliance(){
        return alliance;
    }

    public ALLIANCE_COLOR getCurrentBallColor(){
        return currentBallColor;
    }

    public String toString(){
        String s = "None";
        switch(alliance) {
            case BLUE:
                s = "BLUE";
                break;
            case RED:
                s = "RED";
            default:
                s = "NONE";    
        }
        return s;
    }
}