package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectoryFollower {
    
    private SwerveTrajectory mTrajectory;
    private double prevTime;
    private PIDController mXPID, mYPID;
    private ProfiledPIDController mTurnPID;
    private Pose2d poseRobot = new Pose2d();

    // private HolonomicDriveController mDrivePID;

    public TrajectoryFollower(SwerveTrajectory pTrajectory) {
        mTrajectory = pTrajectory;
        prevTime = 0;
    }

    public ChassisSpeeds follow(Pose2d robotPose, double pTime, boolean targetLock) {
        poseRobot = robotPose;
        ChassisSpeeds desiredSpeeds = mTrajectory.sample(pTime, robotPose);

        if (!Robot.isReal()) {
            simulateRobotPose(mTrajectory.trajectory().sample(pTime).poseMeters, desiredSpeeds);
        }

        prevTime = pTime;
        return desiredSpeeds;
    }

    private void simulateRobotPose(Pose2d pose, ChassisSpeeds desiredSpeeds) {
        Robot.FIELD.setRobotPose(new Pose2d(pose.getTranslation(), new Rotation2d()));

        SmartDashboard.putNumber("Field Relative X Velocity", desiredSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Field Relative Y Velocity", desiredSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Field Relative Omega", desiredSpeeds.omegaRadiansPerSecond);
    }

    public boolean finished() {
        return prevTime >= mTrajectory.trajectory().getTotalTimeSeconds();
    }

}