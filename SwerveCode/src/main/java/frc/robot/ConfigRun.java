//////////////////////////////////////////////////////////////////////////////////////////////////////
// Robot competition configuration
/////////////////////////////////////////////////////////////////////////////////////////////////////

package frc.robot;

public final class ConfigRun {
    // Treat like a static class.  No instantiation
    private ConfigRun(){throw new UnsupportedOperationException();}
    
    //
    // Autonomous Configuration
    //
    // We might want to move the enum definition into the Autonomous class
    public enum AutoOptions {fiveBall, fourBall, threeBall, twoBall, oneBall, move, noAuto} 
    //
    public static final AutoOptions AUTONOMOUS_PROGRAM = AutoOptions.oneBall;

    public static final boolean WAYPOINT = true;
    
    //
    // Driving Power
    //
    public static final double TRANSLATE_POWER = 0.2;      // Scaling for teleop driving.  1.0 is maximum
    public static final double ROTATE_POWER    = 0.2;      // Scaling for teleop driving.  1.0 is maximum

    //
    // Joystick configuration
    //
    public static final double JOYSTICK_DEADBAND = 0.01;    // Deadband for translate and rotate joysticks

    //
    // Maximum current provided to motors to help limit battery drain and harsh turning & acceleration
    //
    public static final double MAX_SWERVE_DRIVE_CURRENT = 20.0; // Lower values will reduce acceleration
    public static final double MAX_SWERVE_STEER_CURRENT = 10.0; // Lower values will turn slower

    //
    // Speeds for autonomous ball collecting
    //
    public static final double TARGET_LOCKED_SPEED = -4;
    public static final double TARGET_CLOSE_SPEED  = -1.0;

    public static final double MAX_TURN_SPEED = Math.PI/2; //max turnspeed in radians
    public static final double MAX_MOVE_SPEED = 1.0; //1.7; //max move speed in meters per second

}
