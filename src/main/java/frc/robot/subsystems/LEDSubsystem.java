package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;
    public int[] currentRGB = {0, 0, 0};
    public boolean rainbowMode;
    public double cycleCount = 0;

    public LEDSubsystem() {
        rainbowMode = false;
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(0);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(14);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        if (rainbowMode) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                if ((i + cycleCount) % m_ledBuffer.getLength() < 3) {
                    m_ledBuffer.setRGB(i, 255, 0, 0); // Set to red
                } else {
                    m_ledBuffer.setRGB(i, 0, 0, 0); // Set to off
                }
            }
            cycleCount = (cycleCount + 0.2) % m_ledBuffer.getLength();
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
