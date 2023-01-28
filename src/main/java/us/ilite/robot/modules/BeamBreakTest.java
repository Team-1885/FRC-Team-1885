package us.ilite.robot.modules;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SensorUtil;
import us.ilite.common.lib.util.FilteredAverage;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.hardware.DigitalBeamSensor;

import java.util.Arrays;

public class BeamBreakTest extends Module {
    private DigitalInput mBeamInput;
    private FilteredAverage mFilter = null;

    private DigitalBeamSensor mDBS;
    public BeamBreakTest(int pInputChannel) {
        mDBS = new DigitalBeamSensor(pInputChannel);
    }
    public BeamBreakTest(int pInputChannel, double pDebounceTime) {
        int numFilters = (int)(pDebounceTime / 0.02);
        double[] gains = new double[numFilters];
        Arrays.fill(gains, 1.0/(double)numFilters);
        mFilter = new FilteredAverage(gains);
        mBeamInput = new DigitalInput(pInputChannel);
    }
    @Override
    public void modeInit(EMatchMode pMode) {

    }
    @Override
    public void readInputs() {
        if(mDBS.isBroken()) {
            db.intake.set(EIntakeData.BEAM_BROKEN, 1.0);
        }
        else {
            db.intake.set(EIntakeData.BEAM_BROKEN, 0.0);
        }
    }
    @Override
    public void setOutputs() {
        isBroken();
    }
    public boolean isBroken() {
        if(mFilter != null) {
            mFilter.addNumber(mBeamInput.get() ? 1.0 :0.0);
            return mFilter.getAverage() <= 0.5;
        } else {
            return mBeamInput.get();
        }
    }
}


