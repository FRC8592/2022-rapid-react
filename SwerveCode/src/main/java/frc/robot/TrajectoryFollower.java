package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TrajectoryFollower {
    
    private Trajectory mTrajectory;
    private double prevTime;
    private PIDController mXPID, mYPID;
    private ProfiledPIDController mTurnPID;
    private Pose2d poseRobot = new Pose2d();

    private double hypotheticalRotation = 0;
    private double currentRotation = 0;

    private Pose2d hubPose = new Pose2d(8.3, 4.125, Rotation2d.fromDegrees(0));

    private HolonomicDriveController mDrivePID;

    public TrajectoryFollower(Trajectory pTrajectory) {
        mTrajectory = pTrajectory;
        prevTime = 0;
        mTurnPID = new ProfiledPIDController(0.01, 0, 0, new Constraints(Math.PI/2, Math.PI/2));
        mXPID = new PIDController(0.01, 0, 0);
        mYPID = new PIDController(0.01, 0, 0);
        mXPID.setTolerance(0.1, 0.1);
        mYPID.setTolerance(0.1, 0.1);
        mTurnPID.setTolerance(0.1, 0.1);
        mDrivePID = new HolonomicDriveController(mXPID, mYPID, mTurnPID);
        if (mTrajectory != null) {
            hypotheticalRotation = mTrajectory.getInitialPose().getRotation().getRadians();
            currentRotation = hypotheticalRotation;
        }

        mTurnPID.enableContinuousInput(-2 * Math.PI, 2 * Math.PI);
    }

    public ChassisSpeeds follow(Pose2d robotPose, double pTime, boolean targetLock) {
        poseRobot = robotPose;

        State state = mTrajectory.sample(pTime);
        // double dt = pTime - prevTime;
        // double dx = state.poseMeters.getX() - robotPose.getX();
        // double dy = state.poseMeters.getY() - robotPose.getY();
        // double turnOutput = mTurnPID.calculate(robotPose.getRotation().getRadians(), getFinalPose().getRotation().getRadians());
        // double xOutput = mXPID.calculate(robotPose.getX(), state.poseMeters.getX());
        // double yOutput = mYPID.calculate(robotPose.getY(), state.poseMeters.getY());

        ChassisSpeeds desiredSpeeds;

        double dt = SmartDashboard.getNumber("Testing/Delta Time", 0.000002);

        if (!Robot.isReal()) {
            simulateRobotPose(state.poseMeters, targetLock);

            desiredSpeeds = mDrivePID.calculate(
                new Pose2d(robotPose.getTranslation(),
                new Rotation2d(currentRotation)), 
                state, 
                getFinalPose().getRotation()
                //new Rotation2d()
            );

            simulateRobotPose(robotPose, state, dt);
        } else {
            desiredSpeeds = mDrivePID.calculate(
                robotPose, 
                state, 
                getFinalPose().getRotation()
            );
        }

        SmartDashboard.putNumber("Field Relative X Velocity", desiredSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Field Relative Y Velocity", desiredSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Field Relative Omega", desiredSpeeds.omegaRadiansPerSecond);

        // currentRotation = hypotheticalRotation;

        return desiredSpeeds;
    }

    private void simulateRobotPose(Pose2d robot, State state, double dt) {
        hypotheticalRotation += mDrivePID.calculate(new Pose2d(robot.getTranslation(), new Rotation2d(currentRotation)), state, getFinalPose().getRotation()).omegaRadiansPerSecond * dt;
    }

    private void simulateRobotPose(Pose2d pose, boolean targetLock) {
        double dx = pose.getX() - hubPose.getX();
        double dy = pose.getY() - hubPose.getY();
        if (targetLock) {
            if (dx >= 0) {
                Robot.FIELD.setRobotPose(new Pose2d(pose.getTranslation(), new Rotation2d(Math.PI + Math.atan(dy/dx))));
            } else {
                Robot.FIELD.setRobotPose(new Pose2d(pose.getTranslation(), new Rotation2d(-2 * Math.PI + Math.atan(dy/dx))));
            }
        } else {
            Robot.FIELD.setRobotPose(new Pose2d(pose.getTranslation(), new Rotation2d(hypotheticalRotation / 180 * Math.PI)));
        }
    }

    public Trajectory getTrajectory() {
        return mTrajectory;
    }

    private Pose2d getFinalPose() {
        return mTrajectory.sample(mTrajectory.getTotalTimeSeconds() - 0.01).poseMeters;
    }

    public boolean finished() {
        Pose2d desiredPose = mTrajectory.sample(mTrajectory.getTotalTimeSeconds()).poseMeters;
        if (prevTime <= mTrajectory.getTotalTimeSeconds()) {
            return false;
        } else if (Math.abs(poseRobot.getX() - desiredPose.getX()) < 0.1 && 
                    Math.abs(poseRobot.getY() - desiredPose.getY()) < 0.1 && 
                    Math.abs(poseRobot.getRotation().getRadians() - desiredPose.getRotation().getRadians()) < 0.1) {

        }
        return prevTime == mTrajectory.getTotalTimeSeconds();
    }

}