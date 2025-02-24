// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class LEDs extends SubsystemBase {
  private final CANdle m_candle = new CANdle(RobotConstants.LED.CANdleId, "CAN");
  private final int LedCount = 150 + 8;

  public String robotState = "";

  @AutoLogOutput(key = "LEDs/Current Color")
  public static String currentColor = "DEFAULT";

  private boolean robotEnabled = false;
  private boolean isRunningCommand = false;

  private boolean bootUpAnimationComplete = false;
  private int step = 0;

  public LEDs() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.5;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
    m_candle.setLEDs(0, 0, 0);
  }

  /* Wrappers so we can access the CANdle from the subsystem */
  public double getVbat() {
    return m_candle.getBusVoltage();
  }

  public double get5V() {
    return m_candle.get5VRailVoltage();
  }

  public double getCurrent() {
    return m_candle.getCurrent();
  }

  public double getTemperature() {
    return m_candle.getTemperature();
  }

  public void configBrightness(double percent) {
    m_candle.configBrightnessScalar(percent, 0);
  }

  public void configLos(boolean disableWhenLos) {
    m_candle.configLOSBehavior(disableWhenLos, 0);
  }

  public void configLedType(LEDStripType type) {
    m_candle.configLEDType(type, 0);
  }

  public void configStatusLedBehavior(boolean offWhenActive) {
    m_candle.configStatusLedState(offWhenActive, 0);
  }

  public void bootUpAnimation() {
    if (bootUpAnimationComplete) return;

    // Clear all LEDs
    // m_candle.setLEDs(0, 0, 0, 0, 0, LedCount);

    // Light up LEDs from both ends inward
    for (int i = 0; i <= step; i++) {
      int leftIndex = i;
      int rightIndex = LedCount - 1 - i;

      // Set color (e.g., warm orange)
      m_candle.setLEDs(255, 15, 0, 0, leftIndex, 1);
      m_candle.setLEDs(255, 15, 0, 0, rightIndex, 1);
    }

    // Move to the next step
    step = step + 2;

    // Check if the animation is complete
    if (step >= LedCount / 2) {
      bootUpAnimationComplete = true;
    }
  }

  public void setIsRunningCommand(boolean flag) {
    isRunningCommand = flag;
  }

  public void setColor(String color) {
    switch (color) {
      case "OFF":
        m_candle.setLEDs(0, 0, 0);
        currentColor = "OFF";
        break;
      case "DEFAULT": // ORANGE
        m_candle.setLEDs(255, 15, 0);
        currentColor = "DEFAULT";
        break;
      case "WHITE":
        m_candle.setLEDs(255, 255, 255, 255, 0, LedCount);
        currentColor = "WHITE";
        break;
      case "CYAN":
        m_candle.setLEDs(0, 40, 255);
        currentColor = "CYAN";
        break;
      case "BLUE":
        m_candle.setLEDs(0, 0, 255);
        currentColor = "CYAN";
        break;
      case "GREEN":
        m_candle.setLEDs(0, 255, 0);
        currentColor = "GREEN";
        break;
      case "RED":
        m_candle.setLEDs(255, 0, 0);
        currentColor = "RED";
        break;
    }
  }

  @Override
  public void periodic() {
    robotEnabled = DriverStation.isEnabled();

    if (bootUpAnimationComplete == false) {
      bootUpAnimation();
    }

    if (currentColor == "DEFAULT") {
      m_candle.setLEDs(255, 15, 0);
    } else if (currentColor == "WHITE") {
      m_candle.setLEDs(255, 255, 255, 255, 0, LedCount);
    } else if (currentColor == "CYAN") {
      m_candle.setLEDs(0, 40, 255);
    }

    if (isRunningCommand == false && robotEnabled) {
      setColor(currentColor);
    }

    m_candle.modulateVBatOutput(1);
  }
}
