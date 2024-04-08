package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;
    public int[] currentRGB = {0, 0, 0};
    public boolean rainbowMode;
    public int direction = 1;
    public int position = 0;
    public int direction2 = 1;
    public int position2 = 19;
    public int delaycounter = 0;

    public LEDSubsystem() {
        rainbowMode = true;
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(0);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(47);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        if (rainbowMode) {
            if (delaycounter < 2) {
                delaycounter++;
                return;
            }
            delaycounter = 0;

            // Move four red LEDs back and forth
            for (int i = 0; i < 19; i++) {
                m_ledBuffer.setRGB(i, 0, 0, 0); // Set to red
            }
            position += direction;
            if (position == 0 || position == 19 - 4) {
                direction *= -1; // Reverse direction when reaching ends
            }
            for (int i = position; i < position + 4; i++) {
                m_ledBuffer.setRGB(i, 255, 0, 0); // Set to red
            }

            // Move five
            for (int i = 19; i < 47; i++) {
                m_ledBuffer.setRGB(i, 0, 0, 0); // Set to red
            }
            position2 += direction2;
            if (position2 == 19 || position2 == 47 - 5) {
                direction2 *= -1; // Reverse direction when reaching ends
            }
            for (int i = position2; i < position2 + 5; i++) {
                m_ledBuffer.setRGB(i, 255, 0, 0); // Set to red
            }

            // Set the LEDs
            m_led.setData(m_ledBuffer);
        } else {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, currentRGB[0], currentRGB[1], currentRGB[2]);
            }
            // Set the LEDs
            m_led.setData(m_ledBuffer);
        }
    }

    public void toggleRainbow() {
        rainbowMode = !rainbowMode;
    }

    public void setColor(int r, int g, int b){
        currentRGB[0] = r;
        currentRGB[1] = g;
        currentRGB[2] = b;
    }
}
