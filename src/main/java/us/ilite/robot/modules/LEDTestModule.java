package us.ilite.robot.modules;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.config.Settings;
import us.ilite.common.types.ELEDControlData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;

public class LEDTestModule extends Module{
    private CANifier mLEDCan;
    private Timer mBlinkTimer;
    public LEDTestModule() {
        this.mBlinkTimer = new Timer();
        this.mBlinkTimer.reset();
        mLEDCan = new CANifier(Settings.HW.CAN.kLEDCanifier);
        mLEDCan.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 20);
        mLEDCan.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 20);
    }
    @Override
    public void modeInit(EMatchMode mode) {
        db.ledcontrol.set(ELEDControlData.LED_STATE, 0.0);
        this.mBlinkTimer.stop();
        this.mBlinkTimer.reset();
    }
    @Override
    public void readInputs() {

    }
    @Override
    public void setOutputs() {
        Enums.LEDColorMode color = db.ledcontrol.get(ELEDControlData.DESIRED_COLOR, Enums.LEDColorMode.class);
        Enums.LEDState mode = db.ledcontrol.get(ELEDControlData.LED_STATE, Enums.LEDState.class);
        if(mode == null || color == null) {
            return;
        }
        switch(mode) {
            case NULL:
            case BLINKING:
                //start timer and turn off if mBlinkTimer < half of cycle time, and on if mBlinkTimer > half of cycle time
                mBlinkTimer.start();
                if (mBlinkTimer.hasElapsed(db.ledcontrol.get(ELEDControlData.BLINK_SPEED) / 2)) {
                    setLEDColor(color.getRed(), color.getGreen(), color.getBlue());
                    if (mBlinkTimer.hasElapsed(db.ledcontrol.get(ELEDControlData.BLINK_SPEED))) {
                        mBlinkTimer.reset();
                    }
                } else {
                    setLEDColor(0, 0, 0);
                }
                break;
            case SOLID:
                setLEDColor(color.getRed(), color.getGreen(), color.getBlue());
                break;
        }
    }
    public void setLEDColor(double r, double g, double b) {
        mLEDCan.setLEDOutput(r / 255, CANifier.LEDChannel.LEDChannelA);
        mLEDCan.setLEDOutput(g / 255, CANifier.LEDChannel.LEDChannelB);
        mLEDCan.setLEDOutput(b / 255, CANifier.LEDChannel.LEDChannelC);
    }

    public void shutdown() {
        mLEDCan.setLEDOutput(0, CANifier.LEDChannel.LEDChannelA);
        mLEDCan.setLEDOutput(0, CANifier.LEDChannel.LEDChannelB);
        mLEDCan.setLEDOutput(0, CANifier.LEDChannel.LEDChannelC);
    }

}
