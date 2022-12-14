package frc.robot;

import java.util.ArrayList;

public class ModuleList {
    private ArrayList<Module> modules;

    public ModuleList() {
        modules = new ArrayList<>();
    }

    public void addModule(Module module) {
        modules.add(module);
    }

    public boolean removeModule(Module module) {
        return modules.remove(module);
    }
}
