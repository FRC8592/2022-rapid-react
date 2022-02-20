//code by Zolton
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

public class ColorSensor{
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    private String alliance;
    private String currentBallColor;


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
        SmartDashboard.putString("alliance", alliance); //current alliance value output the the smartdashboard
        //System.out.println(detectedColor.red);
    }

    public String updateCurrentBallColor(){
        Color detectedColor = m_colorSensor.getColor();
        if(detectedColor.blue > detectedColor.red){
          return "BLUE";
        }else if(detectedColor.red > detectedColor.blue){
          return "RED";
        }else{return "NONE";}
      }

    public boolean compareBallToAlliance(){
        if(currentBallColor.equals(alliance)){
            return true;
        }else if(currentBallColor != alliance){
            return false;
        }else{return false;}
    }

    public int getProximity(){
        //int proximity = m_colorSensor.getProximity();
        return m_colorSensor.getProximity();
    }

    public String getAlliance(){
        return alliance;
    }

    public String getCurrentBallColor(){
        return currentBallColor;
    }
}