package us.ilite.robot.modules;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import us.ilite.common.types.EMatchMode;

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
        gamerColor();
        mLED.setData(mLEDBuffer);
        mLED.start();
        //mTable.getEntry("RGBColor").setNumber();
    }
    /* private void gamerColor() {
        // For every pixel
        for ( var i = 0; i < mLEDBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (mRainbowFirstPixelHue + (i * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        mRainbowFirstPixelHue += 3;
        // Check bounds
        mRainbowFirstPixelHue %= 180;
    }
     */

    private void gamerColor() {
        // For every pixel
        for ( var i = 0; i < mLEDBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var colorPaletteHue = (mRainbowFirstPixelHue + (i * 135 / mLEDBuffer.getLength())) % 256;
            // Set the value
            mLEDBuffer.setHSV(i, colorPaletteHue, 30, 68);
        }
        // Increase by to make the rainbow "move"
        mColorPalettePixelHueOne += 3;
        // Check bounds
        mColorPalettePixelHueOne %= 180;
    }
}

