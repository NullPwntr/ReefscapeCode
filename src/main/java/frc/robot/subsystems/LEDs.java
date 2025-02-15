package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class LEDs extends SubsystemBase {
  private final AddressableLED led = new AddressableLED(RobotConstants.LED.pwmPort);
  private final int length = RobotConstants.LED.ledCount;

  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(length);

  private String ledState = "solid"; // Default state
  private int rainbowOffset = 0; // Rainbow animation offset

  public LEDs() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();

    setState("solid"); // Default to solid color
  }

  /** Sets the LED state, changing its behavior */
  public void setState(String newState) {
    this.ledState = newState;
  }

  /** Returns the LED state */
  @AutoLogOutput(key = "LEDs/State")
  public String getState() {
    return ledState;
  }

  /** Sets all LEDs to a solid color */
  private void setSolidColor(int r, int g, int b) {
    for (int i = 0; i < length; i++) {
      ledBuffer.setRGB(i, r, g, b);
    }
    led.setData(ledBuffer);
  }

  /** Fades LEDs in and out */
  private void breatheEffect(int r, int g, int b, double speed) {
    double brightness = (Math.sin(System.currentTimeMillis() / (1000 / speed)) + 1) / 2.0;
    int adjustedR = (int) (r * brightness);
    int adjustedG = (int) (g * brightness);
    int adjustedB = (int) (b * brightness);
    setSolidColor(adjustedR, adjustedG, adjustedB);
  }

  /** Creates a moving rainbow effect */
  private void rainbowEffect() {
    for (int i = 0; i < length; i++) {
      int hue = (i * 180 / length + rainbowOffset) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    led.setData(ledBuffer);
    rainbowOffset += 3; // Move the rainbow forward
  }

  @Override
  public void periodic() {
    // Use switch-case to handle LED states
    switch (ledState) {
      case "solid":
        setSolidColor(0, 255, 0); // Default to green
        break;
      case "breathe":
        breatheEffect(255, 0, 0, 2.0); // Red breathing effect
        break;
      case "rainbow":
        rainbowEffect();
        break;
      case "off":
        setSolidColor(0, 0, 0); // Turn off LEDs
        break;
      default:
        setSolidColor(255, 255, 255); // White as a fallback
    }
  }
}
