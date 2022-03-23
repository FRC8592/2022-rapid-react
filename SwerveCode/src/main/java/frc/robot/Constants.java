// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;


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

    // CAN IDs for the lift motors
    public static final int LIFT_RIGHT_CAN = 25;
    public static final int LIFT_LEFT_CAN  = 26;

    // Other CAN IDs
    public static final int PDH_CAN = 1;

    // DIO Ports
    public static final int LINE_BREAK_TOP_SENSOR_PORT    = 0;
    public static final int LINE_BREAK_BOTTOM_SENSOR_PORT = 1;
    public static final int COLLECTOR_ARM_LIMIT_SWITCH    = 3;

    // Constants for controlling the shooter flywheel
    public static double FLYWHEEL_VOLTAGE = 11;                // Maximum controller voltage for voltage compensation
    public static double FLYWHEEL_P = 0.155;                    // Starting value.  Needs tuning
    public static double FLYWHEEL_I = 0.0000001;               // Starting value.  Needs tuning
    public static double FLYWHEEL_D = 0.00;                    // Starting value.  Needs tuning
    public static double FLYWHEEL_F = 0.055;                   // Starting value.  Needs tuning
    public static double STARTING_FLYWHEEL_SPEED = 1000;
    public static double REJECT_FLYWHEEL_SPEED   = 500;
    public static double RPM_TO_TICKS_MS = 2048.0 / 600.0;     // Conversion factor for rotational velocity (RPM to ticks per 100ms)
    public static double RPM_MAX_ERROR   = 5;                 // Allowed RPM error for flywheel

    // Vision constants for the ring camera
    public static double RING_LOCK_ERROR       = 2.0;           // Angular error allowed for targetting
    public static double RING_CLOSE_ERROR      = 4.0;           // Closing in on acceptable error
    public static double TURRET_ERROR          = 0.5;           // Allowed aiming error in degrees
    public static double RING_CAMERA_HEIGHT    = 36.75;          // Limelight height above ground (inches)
    public static double RING_CAMERA_ANGLE     = 30.0;          // Limelight camera angle above horizontal (degrees)
    public static double RING_TARGET_HEIGHT    = 104.0;         // Center of target above ground (inches)
    public static double TURRET_ROTATE_KP      = 6.0;           // Proportional constant for rotate speed
    public static String LIMELIGHT_RING        = "limelight-ring";

    // Vision constants for the ball camera
    public static double BALL_LOCK_ERROR       = 4.0;
    public static double BALL_CLOSE_ERROR      = 7.0;           // Closing in on acceptable error
    public static double BALL_ERROR            = 0.5;           // Allowed aiming error in degrees
    public static double BALL_CAMERA_HEIGHT    = 35.75;
    public static double BALL_CAMERA_ANGLE     = 30.0;
    public static double BALL_TARGET_HEIGHT    = 4.75;
    public static double BALL_ROTATE_KP        = 4.0;           // Proportional constant for turret rotate speed
    public static String LIMELIGHT_BALL        = "limelight-ball";

    // Common vision constants
    public static double MIN_TURN_SPEED = 0.05;

    // Color sensor
    public static int MIN_BALL_PROXIMITY = 300;
    public static int MAX_COLOR_CHECKS   = 10;

    // Main collector
    public static double COLLECT_PROCESSING_POWER =  0.3;
    public static double COLLECT_STAGING_POWER    =  0.3;
    public static double UNJAM_PROCESSING_POWER   = -0.2;
    public static double UNJAM_STAGING_POWER      = -0.2;
    public static double SHOOT_STAGING_POWER      =  1.0;

    // Collector arm
    public static int BALL_SET_POINT = -3100;   // -3200 is bottom
    public static double ARM_UP_P = 0.12;       // Starting value.  Needs tuning
    public static double ARM_UP_I = 0.0001;     // Starting value.  Needs tuning
    public static double ARM_UP_D = 0.0;        // Starting value.  Needs tuning
    public static double ARM_UP_F = 0.0;        // Starting value.  Needs tuning

    public static double ARM_DOWN_P = 0.096;    // Starting value.  Needs tuning
    public static double ARM_DOWN_I = 0.0;      // Starting value.  Needs tuning
    public static double ARM_DOWN_D = 12.0;     // Starting value.  Needs tuning
    public static double ARM_DOWN_F = 0.0;      // Starting value.  Needs tuning

    public static int    ARM_MM_SMOOTHING   = 1;
    public static double ARM_MM_CRUISE_VELO = 300;
    public static double ARM_MM_ACCEL       = 2400;
    public static double ARM_DEADBAND       = 0.001;  // Set very small.  Default is 0.04
    public static double ARM_STEADY_POWER   = 0.10;
    public static int    ARM_TICKS_180      = 4736;

    // Constants for the lift
    public static double LIFT_VOLTAGE = 11;     // Maximum controller voltage for voltage compensationble 
    public static SupplyCurrentLimitConfiguration LIFT_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 40, 60, 0.5);

    public static int LIFT_TOP_POINT = 0;   // Need to measure top
    public static double LIFT_UP_P = 0.0;   // Starting value.  Needs tuning
    public static double LIFT_UP_I = 0.0;   // Starting value.  Needs tuning
    public static double LIFT_UP_D = 0.0;   // Starting value.  Needs tuning
    public static double LIFT_UP_F = 0.0;   // Starting value.  Needs tuning

    public static double LIFT_DOWN_P = 0.0;     // Starting value.  Needs tuning
    public static double LIFT_DOWN_I = 0.0;     // Starting value.  Needs tuning
    public static double LIFT_DOWN_D = 0.0;     // Starting value.  Needs tuning
    public static double LIFT_DOWN_F = 0.0;     // Starting value.  Needs tuning

    public static int    LIFT_MM_SMOOTHING   = 1;
    public static double LIFT_MM_CRUISE_VELO = 300;
    public static double LIFT_MM_ACCEL       = 2400;
    public static double LIFT_DEADBAND       = 0.001;  // Set very small.  Default is 0.04
    public static double LIFT_STEADY_POWER   = 0.10;
    public static int    LIFT_TICKS_180      = 4736;

    // Limit collector arm current to 5A continuous, 20A peak
    public static SupplyCurrentLimitConfiguration ARM_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 5, 20, 0.5);

    // Table for flywheel speeds.  Each entry represents 12" of distance from reflectors
    public static double RANGE_TABLE[] = {2110, 2110, 2110, 2110, 2110, 2110, 2110, 2110, 2150, 2200, 2300, 2390, 2490, 2640, 2765, 2855, 3100, 3200, 3350, 3400};

    // Limelight LED modes
    public static enum LIMELIGHT_LIGHT {PIPELINE_MODE, FORCE_OFF, FORCE_BLINK, FORCE_ON}
}
