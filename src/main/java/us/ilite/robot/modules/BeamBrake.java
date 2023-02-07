package us.ilite.robot.modules;

import edu.wpi.first.hal.DIOJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SensorUtil;
import us.ilite.common.lib.util.FilteredAverage;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.hardware.DigitalBeamSensor;

import java.util.Arrays;

import static us.ilite.common.types.EFeederData.ENTRY_BEAM;

public class BeamBrake extends Module {

    private DigitalBeamSensor mBrake;

    public BeamBrake() {
        DigitalBeamSensor mBrake = new DigitalBeamSensor(4, FeederModule.kDebounceTime);

    }


    public void readInputs() {
        if (mBrake.isBroken() == true) {
            db.intake.set(EIntakeData.BEAM_BROKEN, 1.0);
        }
        else {
            db.intake.set(EIntakeData.BEAM_BROKEN, 0.0);
        }
    }
    public void setOutputs() {
        mBrake.isBroken();

    }



}
