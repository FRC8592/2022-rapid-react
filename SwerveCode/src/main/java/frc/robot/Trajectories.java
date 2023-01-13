package frc.robot;

import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

public enum Trajectories {

    // Cross Line and Balance Charge Station [*Middle Start*][*Blue*](15 pts)

    CROSS_LINE_PARK_1("output/CrossLinePark1.wpilib.json", new Rotation2d(), false),
    CROSS_LINE_PARK_2("output/CrossLinePark2.wpilib.json", new Rotation2d(), false),

    // Score Pre-load and second game piece [*Bottom Start*][*Blue*](13 pts)

    LOW_2_CUBE_1("output/Low2Cube1.wpilib.json", new Rotation2d(), false),
    LOW_2_CUBE_2("output/Low2Cube2.wpilib.json", new Rotation2d(), false),
    

    // Score Pre-load and second game piece and Balance Charge Station [*Bottom Start*][*Blue*](25 pts)

    LOW_2_CUBE_PARK1("output/Low2CubePark1.wpilib.json", new Rotation2d(), false),
    LOW_2_CUBE_PARK2("output/Low2CubePark2.wpilib.json", new Rotation2d(), false),
    LOW_2_CUBE_PARK3("output/Low2CubePark3.wpilib.json", new Rotation2d(), false),

    ;

    private String path;
    private boolean lock;
    private Rotation2d rot;
    private boolean relative;

    Trajectories(String path) {
        this.path = path;
    }

    Trajectories(String path, Rotation2d rot) {
        this.path = path;
        this.rot = rot;
    }

    Trajectories(String path, Rotation2d rot, boolean relative) {
        this.path = path;
        this.relative = relative;
    }

    public String getName() {
        return path;
    }

    public SwerveTrajectory toTrajectory() {
        Trajectory traj = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (Exception e) {
            System.out.println("Unable to open " + traj + " " + Arrays.toString(e.getStackTrace()));
        }

        return new SwerveTrajectory(traj, rot, false);
    }

    public boolean lockToTarget() {
        return lock;
    }

    public static Trajectories parseEnum(SwerveTrajectory trajectory) {
        for (Trajectories traj : Trajectories.values()) {
            if (traj.toTrajectory().equals(trajectory)) {
                return traj;
            }
        }
        return null;
    }
}