package us.ilite.robot.modules;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EClimberData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static us.ilite.common.types.EClimberData.IS_SINGLE_CLAMPED;

public class pneumaticspractice extends Module{
    private final DoubleSolenoid mCLPNSingle;
    private int setValue;

    public pneumaticspractice(){
        mCLPNSingle = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 4, 5);
    }

    @Override
    public void modeInit(EMatchMode mode){
        mCLPNSingle.set(kOff);
    }

    @Override
    public void readInputs(){

    }

    @Override
    public void setOutputs(){
        Enums.EClampMode singleMode = db.climber.get(IS_SINGLE_CLAMPED, Enums.EClampMode.class);
        if (singleMode == Enums.EClampMode.CLAMPED) {
            mCLPNSingle.set(DoubleSolenoid.Value.kForward);
        } else if (singleMode == Enums.EClampMode.RELEASED) {
            mCLPNSingle.set(DoubleSolenoid.Value.kReverse);
        }
    }

}
