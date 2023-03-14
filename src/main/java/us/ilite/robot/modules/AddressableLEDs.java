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
    public AddressableLEDs() {
        mLED = new AddressableLED(9);
        mLEDBuffer = new AddressableLEDBuffer(60);
        mLED.setLength(mLEDBuffer.getLength());
        mLED.setData(mLEDBuffer);
        mLED.start();
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

    int i = 0;
    private void gamerColor() {
        // For every pixel
        if (i < mLEDBuffer.getLength()) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            var hue = (mRainbowFirstPixelHue + (i * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(i, hue, 255, 128);
            hue = (mRainbowFirstPixelHue + ((i+1) * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(i+1, hue, 255, 128);
            hue = (mRainbowFirstPixelHue + ((i+2) * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(i+2, hue, 255, 128);
            hue = (mRainbowFirstPixelHue + ((i+3) * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(i+3, hue, 255, 128);
            hue = (mRainbowFirstPixelHue + ((i+4) * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(i+4, hue, 255, 128);

            i+=5;
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
//            mLEDBuffer.setHSV(i, 30, 255, 200);

            var colorPaletteHue = (mRainbowFirstPixelHue + (i * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            if(i % 2 == 0) {
                colorPaletteHue = 45;
                mLEDBuffer.setHSV(i, colorPaletteHue, 255, 255);
            }
            else {
                colorPaletteHue = 90;
                mLEDBuffer.setHSV(i, colorPaletteHue, 255, 200);
            }
            i ++;
        } else {
            mLED.setData(mLEDBuffer);
            i = 0;
        }


    }

    public void yellowCones() {
        if (i < mLEDBuffer.getLength()) {

            mLEDBuffer.setRGB(i,128,0,128);
            i++;
        }
        else {
            mLED.setData(mLEDBuffer);
            i = 0;

        }

    }
    public void purpleCubes() {
        if (i < mLEDBuffer.getLength()) {
            mLEDBuffer.setRGB(i,255,255,0);
            i++;
        }
        else {
            mLED.setData(mLEDBuffer);
            i = 0;
        }
    }
    public void trackingButtonPressed() {
        if (i < mLEDBuffer.getLength()) {
            mLEDBuffer.setRGB(i,255,0,0);
            i++;
        }
        else {
            mLED.setData(mLEDBuffer);
            i = 0;
        }
    }
}

