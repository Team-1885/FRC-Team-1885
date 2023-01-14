package us.ilite.robot.modules;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;
import static us.ilite.common.types.EClimberData.IS_SINGLE_CLAMPED;

public class PneumaticTest extends Module {
    private DoubleSolenoid mCLPNSingle;
    public PneumaticTest() {
        mCLPNSingle = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 4, 5);
    }
    @Override
    public void modeInit(EMatchMode mode) {

    }
    @Override
    public void readInputs() {
        db.climber.set(IS_SINGLE_CLAMPED, false);
    }
    @Override
    public void setOutputs() {
        Enums.EClampMode singleMode = db.climber.get(IS_SINGLE_CLAMPED, Enums.EClampMode.class);

        if (singleMode == Enums.EClampMode.CLAMPED) {
            mCLPNSingle.set(DoubleSolenoid.Value.kForward);
        } else if (singleMode == Enums.EClampMode.RELEASED) {
            mCLPNSingle.set(DoubleSolenoid.Value.kReverse);
        }
    }
}
