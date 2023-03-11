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
    int i = 0;
    public AddressableLEDs() {
        mLED = new AddressableLED(9);
        mLEDBuffer = new AddressableLEDBuffer(60);
        mLED.setLength(mLEDBuffer.getLength());
        System.out.println("Your mother is muy bonita");
        mTable = NetworkTableInstance.getDefault().getTable("newLED");
        mTable.getEntry("newLED").setString("DEBUG: NOT SET");
    }
    @Override
    public void modeInit(EMatchMode pMode) {

    }
    @Override
    protected void readInputs() {

        mTable.getEntry("newLED").setString("getInputs");
    }
    @Override
    protected void setOutputs() {
        System.out.println("Alan is a poop head");
        mTable.getEntry("newLED").setString("setOutputs");
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
        if(i < 60){
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var colorPaletteHue = (mRainbowFirstPixelHue + (i * 180 / mLEDBuffer.getLength())) % 180;
          //  final var colorPaletteHue = (mRainbowFirstPixelHue + (i * 135 / mLEDBuffer.getLength())) % 256;
            // Set the value
            mLEDBuffer.setHSV(i, colorPaletteHue, 30, 68);
            i++;
        }
        else {
            i = 0;
            System.out.println("else statement");
            // Increase by to make the rainbow "move"
            mColorPalettePixelHueOne += 3;
            // Check bounds
            mColorPalettePixelHueOne %= 180;
        }
    }
}


