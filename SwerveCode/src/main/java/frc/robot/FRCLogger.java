package frc.robot;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.*;

public class FRCLogger {
    private boolean log;
    private String logFolder;

    /**
     * Initialize the logger
     * 
     * @param log       Whether to log or not when a class calls the log function
     * @param logFolder The folder to log to within RealOutputs; used to be
     *                  "CustomLogs"
     */
    public FRCLogger(boolean log, String logFolder) {
        this.log = log;
        this.logFolder = logFolder;
    }

    /**
     * Logs a single object. Acceptable datatypes are boolean, byte, int, double,
     * String, boolean[], byte[], int[], double[], and String[]. Datatypes that are
     * converted into one of the above automatically are:
     * 
     * <p>
     * SwerveModule: Logs the steer angle and drive velocity, in that order, of
     * the SwerveModule. Steer angle is logged in both degrees and radians. Expects
     * input steer angle to be in degrees.
     * 
     * <p>
     * SwerveModule[]: Logs the steer angles and drive velocities, in that order,
     * of each SwerveModule to a double[]. Steer angle is logged in both degrees and
     * radians. Expects input steer angle to be in degrees
     * 
     * <p>
     * Pose2d: Logs the location X, location Y, and rotation, in that order, to
     * a double[]. Rotation is logged in degrees and radians; locations
     * X and Y are logged in meters and inches. Expects input locations X and Y
     * to be in meters.
     * 
     * <p>
     * Rotation2d: Logs the rotation. Rotation is logged in degrees and radians.
     * 
     * @param filePath An Object containing the "folder" that the logged data is
     *                 stored in. Use a String to specify a custom path within the
     *                 log folder, or input {@code this} to automatically name and
     *                 generate the path.
     * @param name     A String containing the name of the logged data. Usually the
     *                 name of the function or variable that is being logged.
     * @param data     An Object of one of the acceptable datatypes listed above
     *                 containing the data to log.
     */
    public void log(Object filePath, String name, Object data) {
        if (log) {
            String path;
            if (filePath.getClass().getSimpleName().equals("String")) {
                path = (String) filePath;
            } else {
                path = filePath.getClass().getSimpleName();
            }

            // Check for the "data" object being one of the supported datatypes and log it.
            if (data.getClass().getSimpleName().equals("boolean"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (boolean) data);
            if (data.getClass().getSimpleName().equals("byte"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (byte) data);
            if (data.getClass().getSimpleName().equals("int"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (int) data);
            if (data.getClass().getSimpleName().equals("double"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (double) data);
            if (data.getClass().getSimpleName().equals("String"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (String) data);
            if (data.getClass().getSimpleName().equals("boolean[]"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (boolean[]) data);
            if (data.getClass().getSimpleName().equals("byte[]"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (byte[]) data);
            if (data.getClass().getSimpleName().equals("int[]"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (int[]) data);
            if (data.getClass().getSimpleName().equals("double[]"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (double[]) data);
            if (data.getClass().getSimpleName().equals("String[]"))
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name, (String[]) data);

            // Now the if-statements that convert datatypes that Advantage* doesn't
            // support into something it does.

            if (data instanceof SwerveModule) {
                SwerveModule swerveModule = (SwerveModule) data;
                double[] resultDegrees = new double[2];
                resultDegrees[0] = swerveModule.getSteerAngle();
                resultDegrees[1] = swerveModule.getDriveVelocity();
                double[] resultRadians = new double[2];
                resultRadians[0] = Math.toRadians(swerveModule.getSteerAngle());
                resultRadians[1] = swerveModule.getDriveVelocity();
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Degrees",
                        resultDegrees);
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Radians",
                        resultRadians);
            }
            if (data instanceof SwerveModule[]) {
                SwerveModule[] swerveModules = (SwerveModule[]) data;
                double[] resultDegrees = new double[2 * swerveModules.length];
                double[] resultRadians = new double[2 * swerveModules.length];
                for (int i = 0; i < swerveModules.length; i++) {
                    resultDegrees[i * 2] = swerveModules[i].getSteerAngle();
                    resultDegrees[i * 2 + 1] = swerveModules[i].getDriveVelocity();
                    resultRadians[i * 2] = Math.toRadians(swerveModules[i].getSteerAngle());
                    resultRadians[i * 2 + 1] = swerveModules[i].getDriveVelocity();
                }
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Degrees",
                        resultDegrees);
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Radians",
                        resultRadians);
            }
            if (data instanceof Pose2d) {
                Pose2d pose = (Pose2d) data;
                double[] resultMetersDegrees = new double[3];
                double[] resultMetersRadians = new double[3];
                double[] resultInchesDegrees = new double[3];
                double[] resultInchesRadians = new double[3];
                resultMetersDegrees[0] = pose.getX();
                resultMetersDegrees[1] = pose.getY();
                resultMetersDegrees[2] = pose.getRotation().getDegrees();
                resultMetersRadians[0] = pose.getX();
                resultMetersRadians[1] = pose.getY();
                resultMetersRadians[2] = pose.getRotation().getRadians();
                resultInchesDegrees[0] = pose.getX() * 39.3701;
                resultInchesDegrees[1] = pose.getY() * 39.3701;
                resultInchesDegrees[2] = pose.getRotation().getDegrees();
                resultInchesRadians[0] = pose.getX() * 39.3701;
                resultInchesRadians[1] = pose.getY() * 39.3701;
                resultInchesRadians[2] = pose.getRotation().getRadians();
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Meters-Degrees",
                        resultMetersDegrees);
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Meters-Radians",
                        resultMetersRadians);
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Inches-Degrees",
                        resultInchesDegrees);
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "/Inches-Radians",
                        resultInchesRadians);
            }
            if (data instanceof Rotation2d) {
                Rotation2d rotation = (Rotation2d) data;
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "Degrees",
                        rotation.getDegrees());
                Logger.getInstance().recordOutput(logFolder + "/" + (String) path + "/" + name + "Radians",
                        rotation.getRadians());
            }
        }
    }
}