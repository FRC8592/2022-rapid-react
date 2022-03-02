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

    //public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    //
    // CAN IDs for the drivertrain motors and CANcoders
    //
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR    = 6;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR    = 5;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER  = 4;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(148.8);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR     = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR     = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER   = 1;
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

    //IR sensor ports
    public static final int LINE_BREAK_TOP_SENSOR_PORT = -2;
    public static final int LINE_BREAK_BOTTOM_SENSOR_PORT = -1;

    public static enum ALLIANCE_COLOR {
        BLUE, RED, NONE
    }

    //ringVision constants
    public static double RING_LOCK_ERROR       = 1.0;
    public static double TURRET_ERROR          = 0.5;           // Allowed aiming error in degrees
    public static double RING_CAMERA_HEIGHT    = 36.0;            // Limelight height above ground (inches)
    public static double RING_CAMERA_ANGLE     = 0.0;           // Limelight camera angle above horizontal (degrees)
    public static double RING_TARGET_HEIGHT    = 104;           // Center of target above ground (inches)
    public static double TURRET_ROTATE_KP      = 15.6;          // Proportional constant for turret rotate speed
    public static String LIMELIGHT_RING        = "limelight-ring";

    //ballVision constants
    public static double BALL_LOCK_ERROR       = 1.0;
    public static double BALL_ERROR            = 0.5;           // Allowed aiming error in degrees
    public static double BALL_CAMERA_HEIGHT    = 0.0;
    public static double BALL_CAMERA_ANGLE     = 0.0;
    public static double BALL_TARGET_HEIGHT    = 0.0;
    public static double BALL_ROTATE_KP = 1.0 / 15.0;   // Proportional constant for turret rotate speed
    public static String LIMELIGHT_BALL = "limelight-ball";
}
