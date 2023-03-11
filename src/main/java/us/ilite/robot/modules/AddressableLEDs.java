package us.ilite.robot.modules;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import us.ilite.common.types.EMatchMode;

public class AddressableLEDs extends Module{
    @Override
    public void modeInit(EMatchMode pMode) {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        AddressableLED m_led = new AddressableLED(9);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values for purple
            m_ledBuffer.setHSV(i, 284, 100, 75);
        }

        m_led.setData(m_ledBuffer);

        purpleOmbre();

    }

    private void purpleOmbre() {
        // For every pixel
        AddressableLEDBuffer m_ledBuffer;
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    @Override
    protected void readInputs() {

    }
    @Override
    protected void setOutputs() {

    }
}

