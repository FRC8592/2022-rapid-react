package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class TrajectoryFollower {
    
    private Trajectory mTrajectory;
    private double prevTime;

    public TrajectoryFollower(Trajectory pTrajectory) {
        mTrajectory = pTrajectory;
        prevTime = 0;
    }

    public ChassisSpeeds follow(Pose2d robotPose, double pTime) {
        State state = mTrajectory.sample(pTime);
        double dt = pTime - prevTime;
        double dx = state.poseMeters.getX() - robotPose.getX();
        double dy = state.poseMeters.getY() - robotPose.getY();
        double dOmega = state.poseMeters.getRotation().getRadians() - robotPose.getRotation().getRadians();

        Robot.FIELD.setRobotPose(state.poseMeters);

        double x = state.poseMeters.getX();
        double y = state.poseMeters.getY();
        Rotation2d rotation = getFinalPose().getRotation();

        Robot.FIELD.setRobotPose(new Pose2d(x, y, rotation));

        prevTime = pTime;
        return new ChassisSpeeds(dx/dt, dy/dt, dOmega/dt);
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