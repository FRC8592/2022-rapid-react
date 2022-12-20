package frc.robot;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.*;
public class FRCLogger {
  private boolean log;
  private String logFolder;
  /**
   * Initialize the logger
   * @param log Whether to log or not when a class calls the log function
   * @param logFolder The folder to log to within RealOutputs; used to be "CustomLogs"
   */
  public FRCLogger(boolean log, String logFolder){
    this.log = log;
    this.logFolder = logFolder;
  }
  /**
   * Logs a single object. Acceptable datatypes are boolean, byte, int, double, String, boolean[], byte[], int[], double[], and String[].
   * 
   * @param path  An Object containing the "folder" that the logged data is stored in. Use a String to specify a custom path within CustomLogs, or input "this" (no quotes) to automatically name and generate the path.
   * @param name  A String containing the name of the logged data. Usually the name of the function or variable that is being logged.
   * @param data  An Object of one of the acceptable datatypes listed above containing the data to log.
   */
  public void log(Object path, String name, Object data){
    if(log){ // If we're logging
      if(path.getClass().getSimpleName().equals("String")){ // If the path is a String
        
        // Check for the "data" object being one of the supported datatypes and log it.
        if(data.getClass().getSimpleName().equals("boolean"))
          Logger.getInstance().recordOutput(logFolder + "/" + (String)path + "/" + name, (boolean)data);
        if(data.getClass().getSimpleName().equals("byte"))
          Logger.getInstance().recordOutput(logFolder + "/" + (String)path + "/" + name, (byte)data);
        if(data.getClass().getSimpleName().equals("int"))
          Logger.getInstance().recordOutput(logFolder + "/" + (String)path + "/" + name, (int)data);
        if(data.getClass().getSimpleName().equals("double"))
          Logger.getInstance().recordOutput(logFolder + "/" + (String)path + "/" + name, (double)data);
        if(data.getClass().getSimpleName().equals("String"))
          Logger.getInstance().recordOutput(logFolder + "/" + (String)path + "/" + name, (String)data);
        if(data.getClass().getSimpleName().equals("boolean[]"))
          Logger.getInstance().recordOutput(logFolder + "/" + (String)path + "/" + name, (boolean[])data);
        if(data.getClass().getSimpleName().equals("byte[]"))
          Logger.getInstance().recordOutput(logFolder + "/" + (String)path + "/" + name, (byte[])data);
        if(data.getClass().getSimpleName().equals("int[]"))
          Logger.getInstance().recordOutput(logFolder + "/" + (String)path + "/" + name, (int[])data);
        if(data.getClass().getSimpleName().equals("double[]"))
          Logger.getInstance().recordOutput(logFolder + "/" + (String)path + "/" + name, (double[])data);
        if(data.getClass().getSimpleName().equals("String[]"))
          Logger.getInstance().recordOutput(logFolder + "/" + (String)path + "/" + name, (String[])data);
      }
      
      else{ // If the path is another object
        
        // Use the name of the inputted object (e.g. Drivetrain) as the folder inside logFolder that the logs are stored in.
        if(data.getClass().getSimpleName().equals("boolean"))
          Logger.getInstance().recordOutput(logFolder + "/" + path.getClass().getSimpleName() + "/" + name, (boolean)data);
        if(data.getClass().getSimpleName().equals("byte"))
          Logger.getInstance().recordOutput(logFolder + "/" + path.getClass().getSimpleName() + "/" + name, (byte)data);
        if(data.getClass().getSimpleName().equals("int"))
          Logger.getInstance().recordOutput(logFolder + "/" + path.getClass().getSimpleName() + "/" + name, (int)data);
        if(data.getClass().getSimpleName().equals("double"))
          Logger.getInstance().recordOutput(logFolder + "/" + path.getClass().getSimpleName() + "/" + name, (double)data);
        if(data.getClass().getSimpleName().equals("String"))
          Logger.getInstance().recordOutput(logFolder + "/" + path.getClass().getSimpleName() + "/" + name, (String)data);
        if(data.getClass().getSimpleName().equals("boolean[]"))
          Logger.getInstance().recordOutput(logFolder + "/" + path.getClass().getSimpleName() + "/" + name, (boolean[])data);
        if(data.getClass().getSimpleName().equals("byte[]"))
          Logger.getInstance().recordOutput(logFolder + "/" + path.getClass().getSimpleName() + "/" + name, (byte[])data);
        if(data.getClass().getSimpleName().equals("int[]"))
          Logger.getInstance().recordOutput(logFolder + "/" + path.getClass().getSimpleName() + "/" + name, (int[])data);
        if(data.getClass().getSimpleName().equals("double[]"))
          Logger.getInstance().recordOutput(logFolder + "/" + path.getClass().getSimpleName() + "/" + name, (double[])data);
        if(data.getClass().getSimpleName().equals("String[]"))
          Logger.getInstance().recordOutput(logFolder + "/" + path.getClass().getSimpleName() + "/" + name, (String[])data);
      }
    }
  }
}
