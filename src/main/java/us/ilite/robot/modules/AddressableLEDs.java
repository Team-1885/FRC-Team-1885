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
    private int mColorPalettePixelHueOne = 0;
    private NetworkTable mTable;
    private int colorIndex;
    public AddressableLEDs() {
        mLED = new AddressableLED(9);
        mLEDBuffer = new AddressableLEDBuffer(60);
        mLED.setLength(mLEDBuffer.getLength());
        mLED.setData(mLEDBuffer);
        mLED.start();
        mTable = NetworkTableInstance.getDefault().getTable("newled");
        colorIndex = 0;
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
        System.out.println(mode);
        System.out.println(db.addressableled.get(EAddressableLEDData.DESIREDCOLOR));
        mTable.getEntry("DesiredColor").setString(mode.toString());
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
            case RED:
                trackingButtonPressed();
                mTable.getEntry("Red").setNumber(db.addressableled.get(EAddressableLEDData.DESIREDCOLOR));
                break;

        }
    }

    private void gamerColor() {
        // For every pixel
        if (colorIndex < mLEDBuffer.getLength()) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            var hue = (mRainbowFirstPixelHue + (colorIndex * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(colorIndex, hue, 255, 128);
            hue = (mRainbowFirstPixelHue + ((colorIndex+1) * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(colorIndex+1, hue, 255, 128);
            hue = (mRainbowFirstPixelHue + ((colorIndex+2) * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(colorIndex+2, hue, 255, 128);
            hue = (mRainbowFirstPixelHue + ((colorIndex+3) * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(colorIndex+3, hue, 255, 128);
            hue = (mRainbowFirstPixelHue + ((colorIndex+4) * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(colorIndex+4, hue, 255, 128);

            colorIndex+=5;
        } else {
            mLED.setData(mLEDBuffer);
            // Increase by to make the rainbow "move"
            mRainbowFirstPixelHue += 3;
            // Check bounds
            mRainbowFirstPixelHue %= 180;
            colorIndex = 0;
        }
    }

    private void battlefieldColor() {
        if (colorIndex < mLEDBuffer.getLength()) {
            if(colorIndex % 2 == 0) {
                mLEDBuffer.setHSV(colorIndex, 45, 255, 255);
            } else {
                mLEDBuffer.setHSV(colorIndex, 90, 255, 200);
            }
            colorIndex ++;
        } else {
            mLED.setData(mLEDBuffer);
            colorIndex = 0;
        }
    }

    public void yellowCones() {
        if (colorIndex < mLEDBuffer.getLength()) {
            mLEDBuffer.setRGB(colorIndex,128,0,128);
            colorIndex++;
        } else {
            mLED.setData(mLEDBuffer);
            colorIndex = 0;
        }
    }
    public void purpleCubes() {
        if (colorIndex < mLEDBuffer.getLength()) {
            mLEDBuffer.setRGB(colorIndex,255,255,0);
            colorIndex++;
        } else {
            mLED.setData(mLEDBuffer);
            colorIndex = 0;
        }
    }
    public void trackingButtonPressed() {
        if (colorIndex < mLEDBuffer.getLength()) {
            mLEDBuffer.setRGB(colorIndex,255,0,0);
            colorIndex++;
        } else {
            mLED.setData(mLEDBuffer);
            colorIndex = 0;
        }
    }
}

