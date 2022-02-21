//////////////////////////////////////////////////////////////////////////////////////////////////////
// Robot competition configuration
/////////////////////////////////////////////////////////////////////////////////////////////////////

package frc.robot;

public final class ConfigRun {
    // Treat like a static class.  No instantiation
    private ConfigRun(){throw new UnsupportedOperationException();}
    
    //
    // Gameplay Configuration
    //
    // We might want to move the enum definition into the Autonomous class
    public enum AutoOptions {fiveBall, fourBall, threeBall, twoBall, oneBall, move, noAuto} 
    //
    public static final AutoOptions AUTONOMOUS_PROGRAM = AutoOptions.oneBall;

    
    //
    // Driving Configuration
    //
    public static final double TRANSLATE_POWER = 0.20;      // Scaling for teleop driving.  1.0 is maximum
    public static final double ROTATE_POWER    = 0.20;      // Scaling for teleop driving.  1.0 is maximum
    //
    public static final double JOYSTICK_DEADBAND = 0.01;    // Deadband for translate and rotate joysticks

}
