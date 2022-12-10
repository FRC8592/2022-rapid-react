package frc.common;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Codex {
    
    private NetworkTable mTable;

    public Codex(String pTable) {
        mTable = NetworkTableInstance.getDefault().getTable("Codex/" + pTable);
        mTable.getEntry("/ ").setString("");
    }

    private void setDouble(String entry, double value) {
        mTable.getEntry(entry).setValue(value);
    }

    private void setBoolean(String entry, boolean value) {
        mTable.getEntry(entry).setValue(value);
    }

    private void setString(String entry, String value) {
        mTable.getEntry(entry).setValue(value);
    }

    public void set(String entry, double value) {
        mTable.getEntry(entry).setDouble(value);
    }

    public double get(String entry) {
        return mTable.getEntry(entry).getDouble(0d);
    }

    public String getName() {
        setString("Name", mTable.toString());
        return mTable.toString();
    }

    public String toString() {
        String path = mTable.toString();
        String[] temps = path.split("/");
        return temps[temps.length - 1];
    }
}
