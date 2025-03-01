package frc.robot.subsystems.Leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private static LED m_instance = new LED();
  // PWM port 9
  // Must be a PWM header, not MXP or DIO
  private final AddressableLED m_led = new AddressableLED(0);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(144);

  // Reuse buffer
  // Default to a length of 60, start empty output
  // Length is expensive to set, so only set it once, then just update data

  private int m_rainbowFirstPixelHue;
  private boolean isRed = true;

  public static LED getInstance() {
    return m_instance;
  }

  public LED() {
    m_led.setLength(m_ledBuffer.getLength());
    for (var i = 0; i < 72; i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 255, 0);
    }
    for (var i = 72; i >= 72 && i < 144; i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 0, 255);
    }
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void cycleRedWhitePattern() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        if ((i / 4) % 2 == 0) {
            m_ledBuffer.setRGB(i, 255, 0, 0); // Red
        } else {
            m_ledBuffer.setRGB(i, 255, 255, 255); // White
        }
    }

    m_led.setData(m_ledBuffer);
}

  public void cycleRedWhite() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        if (isRed) {
            m_ledBuffer.setRGB(i, 255, 0, 0); // Red
        } else {
            m_ledBuffer.setRGB(i, 255, 255, 255); // White
        }
    }
    // Toggle the color for the next cycle
    isRed = !isRed;

    m_led.setData(m_ledBuffer);
}
  public void rainbow() {
    // For every pixel
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

    m_led.setData(m_ledBuffer);
  }

  public void HaveNote() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 80, 80);
    }
    m_led.setData(m_ledBuffer);
  }

  public void noNote() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
    m_led.setData(m_ledBuffer);
  }

  public void target() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 0, 0);
    }
    m_led.setData(m_ledBuffer);
  }

  public void wantNote() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 0, 0);
    }
    m_led.setData(m_ledBuffer);
  }

  public void blue() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 255);
    }
    m_led.setData(m_ledBuffer);
  }
}
