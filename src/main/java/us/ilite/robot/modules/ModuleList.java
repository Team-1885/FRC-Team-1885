package us.ilite.robot.modules;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Robot;

import java.util.LinkedList;
import java.util.List;

public class ModuleList extends Module {

    ILog mLogger = Logger.createLog(ModuleList.class);

    protected List<Module> mModules = new LinkedList<>();

    @Override
    public void modeInit(EMatchMode pMode) {
        mModules.forEach(module -> module.modeInit(pMode));
    }

    @Override
    public void readInputs() {
        if(Robot.mode() == EMatchMode.TEST) {
            mModules.forEach(module -> Robot.CLOCK.report("R-"+module.getClass().getSimpleName(), t->module.readInputs()));
        } else {
            mModules.forEach(module -> module.readInputs());
        }

    }

    @Override
    public void setOutputs() {
        if(Robot.mode() == EMatchMode.TEST) {
            mModules.forEach(module -> Robot.CLOCK.report("R-"+module.getClass().getSimpleName(), t->module.setOutputs()));
        } else {
            mModules.forEach(module -> module.setOutputs());
        }
    }

    @Override
    public void shutdown() {
        mModules.forEach(module -> module.shutdown());
    }

    @Override
    public boolean checkModule() {
        boolean allSuccessful = true;
            for (Module module : mModules) {
                boolean moduleSuccessful = module.checkModule();
            allSuccessful = allSuccessful && moduleSuccessful;
            if (!moduleSuccessful) {
                mLogger.error("Self-check failure for module: ", module.getClass());
            } else {
                mLogger.warn("Self-check success for module: ", module.getClass());
            }
        }

        return allSuccessful;
    }

    public void clearModules() {
        mModules.clear();
    }

    public void addModule(Module pModule) {
        if(pModule == null) {
            throw new IllegalArgumentException("the module is null!");
        }

        mModules.add(pModule);
    }

}
