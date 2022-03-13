// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Treat like a static class.  No instantiation
    private Constants() {throw new UnsupportedOperationException();}

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.553;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.597;

    //
    // CAN IDs for the drivertrain motors and CANcoders
    //
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR    = 6;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR    = 5;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER  = 4;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(148.8);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR     = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR     = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER   = 13;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(79.5);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR     = 9;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR     = 8;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER   = 7;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(134.7);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR     = 12;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR     = 11;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER   = 10;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(233.6);

    //
    // CAN IDs for the ball collector and launching system
    //
    public static final int newFlywheelLeft      = 22;
    public static final int newFlywheelRight     = 20;
    public static final int newFlywheelStaging   = 21;
    public static final int newFlywheelCollector = 23;
    public static final int COLLECTOR_ARM_CAN    = 24;

    // Other CAN IDs
    public static final int PDH_CAN = 1;

    // DIO Ports
    public static final int LINE_BREAK_TOP_SENSOR_PORT    = 0;
    public static final int LINE_BREAK_BOTTOM_SENSOR_PORT = 1;
    public static final int COLLECTOR_ARM_LIMIT_SWITCH    = 3;

    // Constants for controlling the shooter flywheel
    public static double FLYWHEEL_VOLTAGE = 11;                // Maximum controller voltage for voltage compensation
    public static double FLYWHEEL_P = 0.15;                    // Starting value.  Needs tuning
    public static double FLYWHEEL_I = 0.00;                    // Starting value.  Needs tuning
    public static double FLYWHEEL_D = 0.00;                    // Starting value.  Needs tuning
    public static double FLYWHEEL_F = 0.054;                   // Starting value.  Needs tuning
    public static double STARTING_FLYWHEEL_SPEED = 2085;
    public static double REJECT_FLYWHEEL_SPEED   = 500;
    public static double RPM_TO_TICKS_MS = 2048.0 / 600.0;     // Conversion factor for rotational velocity (RPM to ticks per 100ms)
    public static double RPM_MAX_ERROR   = 8;                 // Allowed RPM error for flywheel

    // Vision constants for the ring camera
    public static double RING_LOCK_ERROR       = 2.0;           // Angular error allowed for targetting
    public static double RING_CLOSE_ERROR      = 4.0;           // Closing in on acceptable error
    public static double TURRET_ERROR          = 0.5;           // Allowed aiming error in degrees
    public static double RING_CAMERA_HEIGHT    = 35.0;          // Limelight height above ground (inches)
    public static double RING_CAMERA_ANGLE     = 27.0;           // Limelight camera angle above horizontal (degrees)
    public static double RING_TARGET_HEIGHT    = 104.0;         // Center of target above ground (inches)
    public static double TURRET_ROTATE_KP      = 7.0;           // Proportional constant for rotate speed
    public static String LIMELIGHT_RING        = "limelight-ring";

    // Vision constants for the ball camera
    public static double BALL_LOCK_ERROR       = 3.0;
    public static double BALL_CLOSE_ERROR      = 7.0;           // Closing in on acceptable error
    public static double BALL_ERROR            = 0.5;           // Allowed aiming error in degrees
    public static double BALL_CAMERA_HEIGHT    = 34.0;
    public static double BALL_CAMERA_ANGLE     = 27.0;
    public static double BALL_TARGET_HEIGHT    = 4.75;
    public static double BALL_ROTATE_KP        = 8.0;           // Proportional constant for turret rotate speed
    public static String LIMELIGHT_BALL        = "limelight-ball";

    // Common vision constants
    public static double MIN_TURN_SPEED = 1.0;
    public static double MAX_TURN_SPEED = 1.0;

    // Color sensor
    public static int MIN_BALL_PROXIMITY = 300;

    // Main collector
    public static double COLLECT_PROCESSING_POWER =  0.2;
    public static double COLLECT_STAGING_POWER    =  0.2;
    public static double UNJAM_PROCESSING_POWER   = -0.2;
    public static double UNJAM_STAGING_POWER      = -0.2;
    public static double SHOOT_STAGING_POWER      =  1.0;

    // Collector arm
    public static int BALL_SET_POINT = -3100;   // -3200 is bottom
    public static double ARM_UP_P = 0.15;       // Starting value.  Needs tuning
    public static double ARM_UP_I = 0.0001;       // Starting value.  Needs tuning
    public static double ARM_UP_D = 5.0;        // Starting value.  Needs tuning
    public static double ARM_UP_F = 11.0;       // Starting value.  Needs tuning

    public static double ARM_DOWN_P = 0.23;        // Starting value.  Needs tuning
    public static double ARM_DOWN_I = 0.0001;        // Starting value.  Needs tuning
    public static double ARM_DOWN_D = 9.0;        // Starting value.  Needs tuning
    public static double ARM_DOWN_F = -0.11;       // Starting value.  Needs tuning

    //Table for flywheel speeds
    public static double RANGE_TABLE[] = {2110, 2110, 2110, 2110, 2110, 2110, 2110, 2110, 2150, 2200, 2300, 2390, 2490, 2640, 2765, 2855, 3100, 3200, 3350, 3400};

}
