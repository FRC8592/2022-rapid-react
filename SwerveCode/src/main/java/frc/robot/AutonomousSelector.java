package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousSelector {

    public ShuffleboardTab autonTab = Shuffleboard.getTab("Auton Configuration");
    private SendableChooser<Enum> chooser = new SendableChooser<>();

    public enum Autons {
        CrossLinePark,
        Low2Cube,
        Low2CubePark
    }

    private Enum[] mAutons = Autons.values();

    public AutonomousSelector() {
        chooser.setDefaultOption("DEFAULT - CrossLinePark", mAutons[0]);
        for (Enum auto : mAutons) {
            chooser.addOption(auto.name(), auto);
        }

        autonTab.add("Choose Autonomous", chooser)
            .withPosition(3, 3)
            .withSize(4, 2);
    }

    public Autons getSelectedAutonomous() {
        try {
            SmartDashboard.putString("SELECTED AUTONOMOUS NAME", mAutons[chooser.getSelected().ordinal()].name());
            return Autons.values()[chooser.getSelected().ordinal()];
        } catch (Exception e) {
            return null;
        }
    }
}
