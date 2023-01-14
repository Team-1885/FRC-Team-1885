package us.ilite.robot.modules;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EMatchMode;
import us.ilite.logging.Log;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EClimberData.*;

public class Pnuematics extends Module {
    private final DoubleSolenoid mCLPNSingle;


    public Pnuematics() {
        mCLPNSingle = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, 4, 5);
    }
    public void modeInit(EMatchMode mode) {

    }
    public void readInputs() {
        db.climber.set(IS_SINGLE_CLAMPED, true);
    }
    public void setOutputs() {
        Enums.EClampMode doubleMode = db.climber.get(IS_DOUBLE_CLAMPED, Enums.EClampMode.class);
        Enums.EClampMode singleMode = db.climber.get(IS_SINGLE_CLAMPED, Enums.EClampMode.class);
        if (singleMode == Enums.EClampMode.CLAMPED) {
            System.out.println("MY Button got pushed#######################################");


            mCLPNSingle.set(DoubleSolenoid.Value.kForward);
        } else if (singleMode == Enums.EClampMode.RELEASED) {
            mCLPNSingle.set(DoubleSolenoid.Value.kReverse);
        }
    }
}

