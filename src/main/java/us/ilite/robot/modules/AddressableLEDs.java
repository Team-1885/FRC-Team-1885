package us.ilite.robot.modules;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import us.ilite.common.types.EAddressableLEDData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;

public class AddressableLEDs extends Module{
    private AddressableLED mLED;
    private AddressableLEDBuffer mLEDBuffer;
    private int mRainbowFirstPixelHue = 0;
    private NetworkTable mTable;
    public AddressableLEDs() {
        mLED = new AddressableLED(9);
        mLEDBuffer = new AddressableLEDBuffer(60);
        mLED.setLength(mLEDBuffer.getLength());
        mTable = NetworkTableInstance.getDefault().getTable("newled");
    }
    @Override
    public void modeInit(EMatchMode pMode) {

    }
    @Override
    protected void readInputs() {

    }
    @Override
    protected void setOutputs() {
        Enums.EAddressableLEDState mode = db.addressableled.get(EAddressableLEDData.DESIREDCOLOR, Enums.EAddressableLEDState.class);
        mTable.getEntry("DesiredColor").setNumber(db.addressableled.get(EAddressableLEDData.DESIREDCOLOR));
        switch(mode) {
            case GAMER_COLOR:
                gamerColor();
                mTable.getEntry("GamerColor").setNumber(db.addressableled.get(EAddressableLEDData.DESIREDCOLOR));
                break;
            case BATTLEFIElD_COLOR:
                battlefieldColor();
                mTable.getEntry("BattlefieldColor").setNumber(db.addressableled.get(EAddressableLEDData.DESIREDCOLOR));
                break;
            case YELLOW:
                yellowCones();
                mTable.getEntry("Yellow").setNumber(db.addressableled.get(EAddressableLEDData.DESIREDCOLOR));
                break;
            case PURPLE:
                purpleCubes();
                mTable.getEntry("Purple").setNumber(db.addressableled.get(EAddressableLEDData.DESIREDCOLOR));
                break;
        }
        mLED.setData(mLEDBuffer);
        mLED.start();
        //mTable.getEntry("RGBColor").setNumber();
    }
    int i = 0;
    private void gamerColor() {
        // For every pixel
        if (i < mLEDBuffer.getLength()) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (mRainbowFirstPixelHue + (i * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(i, hue, 255, 128);
            i++;
        } else {
            mLED.setData(mLEDBuffer);
            // Increase by to make the rainbow "move"
            mRainbowFirstPixelHue += 3;
            // Check bounds
            mRainbowFirstPixelHue %= 180;
            i = 0;
        }
    }
    private void battlefieldColor() {
        // For every pixel
        if (i < mLEDBuffer.getLength()) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (mRainbowFirstPixelHue + (i * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(i, hue, 255, 128);
            i++;
        }
        else {
            mLED.setData(mLEDBuffer);
            // Increase by to make the rainbow "move"
            mRainbowFirstPixelHue += 3;
            // Check bounds
            mRainbowFirstPixelHue %= 180;
            i = 0;
        }
    }

    public void yellowCones() {
        if (i < mLEDBuffer.getLength()) {
            mLEDBuffer.setRGB(i,255,255,0);
            i++;
        }
        else {
            mLED.setData(mLEDBuffer);
            i = 0;

        }

    }
    public void purpleCubes() {
        if (i < mLEDBuffer.getLength()) {
            mLEDBuffer.setRGB(i,128,0,128);
            i++;
        }
        else {
            mLED.setData(mLEDBuffer);
            i = 0;
        }
    }
}

