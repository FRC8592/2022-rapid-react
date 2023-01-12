package frc.robot;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

public enum Trajectories {
    // B_TO_C("output/B_to_C.wpilib.json"),
    // C_TO_SHOOT("output/C_to_Shoot.wpilib.json"),
    // SHOOT_TO_A("output/Shoot_to_A.wpilib.json"),
    // TARMAC_TO_B("output/Tarmac_to_B.wpilib.json"),
    TEST_PATH("output/Test_Path.wpilib.json"),
    NEXT_PATH("output/Next_Path.wpilib.json"),
    BALL_PATH("output/Ball_Path.wpilib.json"),
    EXTRA_AUTO("output/Example_Path.wpilib.json"),
    GRID_TO_FIRST_CUBE("output/Grid_to_First_Cube.wpilib.json"),
    FIRST_CUBE_TO_GRID("output/First_Cube_to_Grid.wpilib.json"),
    GRID_TO_CLIMB("output/Grid_to_Climb.wpilib.json"),
    TEST_TURN("output/Test_Turn.wpilib.json"),
    Straight_Path("output/STRAIGHT_PATH.wpilib.json")
    ;

    private String path;
    private boolean lock;
    Trajectories(String path) {
        this.path = path;
    }

    public String getName() {
        return path;
    }

    public Trajectory toTrajectory(boolean targetLock) {
        lock = targetLock;
        Trajectory traj = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (Exception e) {
            System.out.println("Unable to open " + traj + " " + Arrays.toString(e.getStackTrace()));
        }
        return traj;
    }

    private Trajectory toTrajectory() {
        Trajectory traj = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (Exception e) {
            System.out.println("Unable to open " + traj + " " + Arrays.toString(e.getStackTrace()));
        }
        return traj;
    }

    public boolean lockToTarget() {
        return lock;
    }

    public static Trajectories parseEnum(Trajectory trajectory) {
        for (Trajectories traj : Trajectories.values()) {
            if (traj.toTrajectory().equals(trajectory)) {
                return traj;
            }
        }
        return null;
    }
}