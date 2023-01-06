package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

public class PeriodicOI {
    public enum ControllerType {
        XBOX,
        PLAYSTATION,
        JOYSTICK
    }

    public enum DriverType {
        DRIVER,
        OPERATOR
    }

    private XboxController driverXBOX, operatorXBOX;
    private PS4Controller driverPS4, operatorPS4;

    private ControllerType mDriverType, mOperatorType;

    public PeriodicOI(ControllerType pDriver, ControllerType pOperator) {
        mDriverType = pDriver;
        mOperatorType = pOperator;
        driverXBOX = new XboxController(0);
    }

    public double [] getLeftJoystick(DriverType type) {
        switch(type) {
            case DRIVER:
                switch(mDriverType) {
                    case XBOX:
                        return new double[] {0, 0};
                    case PLAYSTATION:
                        break;
                    case JOYSTICK:
                        break;
                }
                break;
            case OPERATOR:
                break;
        }
        return new double[] {0,0};
    }

    public double [] getRightJoystick(DriverType type) {
        switch(type) {
            case DRIVER:
                break;
            case OPERATOR:
                break;
        }
        return new double[] {0,0};
    }
}