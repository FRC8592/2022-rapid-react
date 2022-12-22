package frc.robot;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;

public class TrajectoryQueue {
    private Queue<Trajectory> mQueue;
    private Timer mTimer;

    public TrajectoryQueue(Trajectory ... trajectories) {
        mQueue = new LinkedList<Trajectory>();
        for (Trajectory trajectory : trajectories) {
          mQueue.add(trajectory);
        }

        mTimer = new Timer();
    }

    public void configTrajectories(TrajectoryConfig config) {
        for (Trajectory traj : mQueue) {
            Pose2d start = traj.getInitialPose();
            Pose2d end = traj.sample(traj.getTotalTimeSeconds() - 0.02).poseMeters;
            List<Translation2d> interior = new ArrayList<>();
            List<State> states = traj.getStates();
            states.remove(0);
            states.remove(states.size() - 1);
            for (State state : states) {
                interior.add(state.poseMeters.getTranslation());
            }
            traj = TrajectoryGenerator.generateTrajectory(start, interior, end, config);
        }
    }

    public boolean isFinished() {
        return mQueue.size() == 0;
    }

    public boolean isTrajectoryComplete() {
        return mTimer.get() >= currentTrajectory().getTotalTimeSeconds();
    }

    public Trajectory currentTrajectory() {
        mTimer.start();
        return mQueue.peek();
    }

    public Trajectory nextTrajectory() {
        mTimer.reset();
        mTimer.start();
        mQueue.poll();
        return mQueue.peek();
    }

    public int size() {
        return mQueue.size();
    }

    public boolean lockToTarget() {
        return Trajectories.parseEnum(mQueue.peek()).lockToTarget();
    }
}
