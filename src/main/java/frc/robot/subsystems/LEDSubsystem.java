package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private final Notifier loadingNotifier;
    private int direction = 1;
    private int position = 0;
    private int direction2 = 1;
    private int position2 = 19;
    private int delaycounter = 0;

    public LEDSubsystem() {
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(1);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(46);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        loadingNotifier = new Notifier(
                () -> {
                    synchronized (this) {
                        blink(Color.kWhite);
                        m_led.setData(m_ledBuffer);
                    }
                });
        loadingNotifier.startPeriodic(0.02);

        if (RobotController.getBatteryVoltage() < 12.1) {
            isLowBattery = true;
        } else {
            isLowBattery = false;
        }
    }

    public boolean isLowBattery = false;
    public boolean isAutodrive = false;
    public boolean isNote = false;
    public boolean isBoost = false;
    public boolean isInRange = false;
    public boolean isAccelerating = false;
    public boolean isReady = false;

    @Override
    public void periodic() {
        loadingNotifier.stop();

        if (isReady) {
            solid(Color.kGreen);
        } else if (isAccelerating) {
            blink(Color.kGreen);
        } else if (isInRange) {
            solid(Color.kBlue);
        } else if (isBoost) {
            blink(Color.kMediumPurple);
        } else if (isAutodrive) {
            blink(Color.kRed);
        } else if (isNote) {
            solid(new Color(222, 49, 0));
        } else if (isLowBattery) {
            solid(Color.kCrimson);
        } else {
            wave();
        }

        m_led.setData(m_ledBuffer);
    }

    private void wave() {
        if (delaycounter < 2) {
            delaycounter++;
            return;
        }
        delaycounter = 0;

        // Move four red LEDs back and forth
        for (int i = 0; i < 18; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0); // Set to red
        }
        position += direction;
        if (position == 0 || position == 18 - 4) {
            direction *= -1; // Reverse direction when reaching ends
        }
        for (int i = position; i < position + 4; i++) {
            m_ledBuffer.setRGB(i, 255, 0, 0); // Set to red
        }

        // Move five
        for (int i = 18; i < 46; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0); // Set to red
        }
        position2 += direction2;
        if (position2 == 18 || position2 == 46 - 5) {
            direction2 *= -1; // Reverse direction when reaching ends
        }
        for (int i = position2; i < position2 + 5; i++) {
            m_ledBuffer.setRGB(i, 255, 0, 0); // Set to red
        }
    }

    private void blink(Color color) {
        boolean c10n = (Timer.getFPGATimestamp() % 0.2) / 0.2 > 0.5;
        solid(c10n ? Color.kBlack : color);
    }

    private void solid(Color color) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, color);
        }
    }
}
