package us.ilite.robot.modules;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.input.ELogitech310;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import us.ilite.common.config.Settings;
import us.ilite.robot.Enums;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;

public class NewIntakeModule extends Module {
    private final NetworkTable mIntakeTable;
    private final NetworkTable mButtonTable;
    private final DoubleSolenoid mCLPNSingle;


    public NewIntakeModule() {
        mCLPNSingle = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 0, 0); // channels are temporarily set to 0

        mIntakeTable = NetworkTableInstance.getDefault().getTable("intake");
        mButtonTable = NetworkTableInstance.getDefault().getTable("button being pressed");
    }

    @Override
    public void modeInit(EMatchMode pMode){
        mCLPNSingle.set(kOff);
    }

    @Override
    public void readInputs(){

    }

    @Override
    public void setOutputs(){
        Enums.EIntakeState singleMode = db.intake.get(EIntakeData.PNEUMATIC_STATE, Enums.EIntakeState.class);
        setIntakeTableValue("extended", Enums.EIntakeState.EXTENDED);
        setIntakeTableValue("retracted", Enums.EIntakeState.RETRACTED);

        setButtonTableValue("a_btn", ELogitech310.A_BTN);
    }

    private void setIntakeTableValue(String pEntry, Enums.EIntakeState pEnum){
        mIntakeTable.getEntry(pEntry).setNumber(pEnum.ordinal());
    }

    private void setButtonTableValue(String pEntry, ELogitech310 pEnum){
        mButtonTable.getEntry(pEntry).setNumber(db.driverinput.get(pEnum));
    }

}
