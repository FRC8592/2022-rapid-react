package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TrajectoryFollower {
    
    private Trajectory mTrajectory;
    private double prevTime;
    private PIDController mTurnPID, mXPID, mYPID;

    private Pose2d hubPose = new Pose2d(8.3, 4.125, Rotation2d.fromDegrees(0));

    public TrajectoryFollower(Trajectory pTrajectory) {
        mTrajectory = pTrajectory;
        prevTime = 0;
        mTurnPID = new PIDController(0.0001, 0, 0);
        mXPID = new PIDController(0.01, 0, 0);
        mYPID = new PIDController(0.01, 0, 0);
        mXPID.setTolerance(0.1, 0.1);
        mYPID.setTolerance(0.1, 0.1);
        mTurnPID.setTolerance(0.1, 0.1);
    }

    public ChassisSpeeds follow(Pose2d robotPose, double pTime, boolean targetLock) {
        State state = mTrajectory.sample(pTime);
        double dt = pTime - prevTime;
        double dx = state.poseMeters.getX() - robotPose.getX();
        double dy = state.poseMeters.getY() - robotPose.getY();
        double turnOutput = mTurnPID.calculate(robotPose.getRotation().getRadians(), getFinalPose().getRotation().getRadians());
        double xOutput = mXPID.calculate(robotPose.getX(), state.poseMeters.getX());
        double yOutput = mYPID.calculate(robotPose.getY(), state.poseMeters.getY());

        if (!Robot.isReal()) {
            simulateRobotPose(state.poseMeters, targetLock);
        }
    
        prevTime = pTime;
        // return new ChassisSpeeds(dx/dt, dy/dt, turnOutput);
        return new ChassisSpeeds(xOutput, yOutput, turnOutput);
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
            Robot.FIELD.setRobotPose(new Pose2d(pose.getTranslation(), getFinalPose().getRotation()));
        }
        
    }

    public Trajectory getTrajectory() {
        return mTrajectory;
    }

    private Pose2d getFinalPose() {
        return mTrajectory.sample(mTrajectory.getTotalTimeSeconds() - 0.01).poseMeters;
    }

    public boolean finished() {
        return prevTime == mTrajectory.getTotalTimeSeconds();
    }

}