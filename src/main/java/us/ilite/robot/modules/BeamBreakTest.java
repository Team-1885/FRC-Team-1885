package us.ilite.robot.modules;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    private final NetworkTable mTable;

    public BeamBreakTest(int pInputChannel) {
        mDBS = new DigitalBeamSensor(pInputChannel);
        mTable = NetworkTableInstance.getDefault().getTable("why");
    }
    public BeamBreakTest(int pInputChannel, double pDebounceTime) {
        int numFilters = (int)(pDebounceTime / 0.02);
        double[] gains = new double[numFilters];
        Arrays.fill(gains, 1.0/(double)numFilters);
        mFilter = new FilteredAverage(gains);
        mBeamInput = new DigitalInput(pInputChannel);
        mTable = NetworkTableInstance.getDefault().getTable("why");
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
        mTable.getEntry("BEAM_BROKEN").setNumber(db.intake.get(EIntakeData.BEAM_BROKEN));
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


