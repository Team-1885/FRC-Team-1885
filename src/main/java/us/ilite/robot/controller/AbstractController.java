package us.ilite.robot.controller;

import com.flybotix.hfr.codex.RobotCodex;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.*;




import static us.ilite.common.types.drive.EDriveData.*;

import us.ilite.common.lib.util.XorLatch;
import us.ilite.common.types.*;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.Clock;

import static us.ilite.robot.Enums.*;

import java.util.List;

public abstract class AbstractController {
    protected final Data db = Robot.DATA;
    protected final Clock clock = Robot.CLOCK;
    protected boolean mEnabled = false;
    protected int mCycleCount = 0;
    protected double mLastTime = 0d;
    protected double dt = 1d;

    protected boolean mIsBallAdded = false;
    protected boolean mIsBallOut = false;
    protected int mNumBalls = 0;
    protected int mBallsShot = 0;
    protected XorLatch mEntryGate  = new XorLatch();
    protected XorLatch mExitGate = new XorLatch();
    protected Timer mShotTimer = new Timer();
    protected boolean mFireWanted = false;

    //Was 5000, lowered it to 4500 before match 42.
    //After match 42, we lowered it even further by 1000rpm
    protected double mFeederFireSpeed = 3500d;

    public AbstractController(){
        super();
    }







    protected boolean climberWithinTolerance(double pTolerance, double currentAngle, Enums.EClimberAngle pAngle) {
        return Math.abs((currentAngle) - pAngle.getAngle()) < pTolerance;
    }

    /**
     * Enables / Disables this controller.
     * @param pEnabled TRUE if enabled
     */
    public final void setEnabled(boolean pEnabled) {
        mCycleCount = 0;
        mEnabled = pEnabled;
    }

    protected void stopDrivetrain() {
        db.drivetrain.set(EDriveData.STATE, EDriveState.PERCENT_OUTPUT);
        db.drivetrain.set(DESIRED_THROTTLE_PCT, 0.0);
        db.drivetrain.set(DESIRED_TURN_PCT, 0.0);
    }



    protected abstract void updateImpl();













    protected void setLED(Enums.LEDColorMode pColor, Enums.LEDState pState) {
        //pState is in seconds
        db.ledcontrol.set(ELEDControlData.DESIRED_COLOR, pColor);
        db.ledcontrol.set(ELEDControlData.LED_STATE, pState);
        if(pState == LEDState.BLINKING) {
            db.ledcontrol.set(ELEDControlData.BLINK_SPEED, 0.5);
        }
    }
    public void setLED(Enums.LEDColorMode pColor, double pBlinkRate) {
        //pState is in seconds
        db.ledcontrol.set(ELEDControlData.DESIRED_COLOR, pColor);
        db.ledcontrol.set(ELEDControlData.LED_STATE, LEDState.BLINKING);
        db.ledcontrol.set(ELEDControlData.BLINK_SPEED,pBlinkRate);
    }

    /**
     * Provides a way to report on what is used in our codex vs not used. This should help reduce the
     * amount of raw null values we log.
     * @return
     */
    public StringBuilder getUnusedCodexReport() {
        StringBuilder unused = new StringBuilder();
        for(RobotCodex rc : db.mMappedCodex.values()) {
            StringBuilder sb = new StringBuilder();
            int count = 0;
            List<Enum> enums = rc.meta().enums();
            for(Enum e : enums) {
                if(rc.isNull(e)) {
                    sb.append(e.name()).append(",");
                    count++;
                }
            }
            if(count > 0 ) {
                unused.append("Codex ").
                        append(rc.meta().getEnum().getSimpleName()).
                        append(" has empty values at ").
                        append(sb).
                        append("\n");
            }
        }
        return unused;
    }

}