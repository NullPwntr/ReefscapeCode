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

public class LEDs extends SubsystemBase {
  private final CANdle m_candle = new CANdle(RobotConstants.LED.CANdleId, "CAN");
  private final int LedCount = 150 + 8;

  public String robotState = "";
  public String currentColor = "";
  private boolean robotEnabled = false;
  private boolean isRunningCommand = false;

  
  private boolean bootUpAnimationComplete = false;
  private int step = 0;

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
  }

  public LEDs() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.5;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);

    bootUpAnimation(); // runs once
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
    m_candle.setLEDs(0, 0, 0, 0, 0, LedCount);

    // Light up LEDs from both ends inward
    for (int i = 0; i <= step; i++) {
        int leftIndex = i;
        int rightIndex = LedCount - 1 - i;

        // Set color (e.g., warm orange)
        m_candle.setLEDs(255, 100, 0, 0, leftIndex, 1);
        m_candle.setLEDs(255, 100, 0, 0, rightIndex, 1);
    }

    // Move to the next step
    step++;

    // Check if the animation is complete
    if (step >= LedCount / 2) {
      bootUpAnimationComplete = true;
    }
}

  public void setIsRunningCommand(boolean flag){
    isRunningCommand = flag;
  }

  public void setColor(String color){
    switch (color) {
      case "OFF":
        m_candle.setLEDs(0, 0, 0);
        currentColor = "OFF";
        break;
      case "DEFAULT": // ORANGE
        m_candle.setLEDs(255, 30, 0);
        currentColor = "DEFAULT";
        break;
      case "WHITE":
        m_candle.setLEDs(255, 255, 255);
        currentColor = "WHITE";
        break;
      case "CYAN":
        m_candle.setLEDs(0, 170, 255);
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

    if(isRunningCommand == false && bootUpAnimationComplete && robotEnabled){
      setColor("DEFAULT");
    }

    if(robotEnabled == false && bootUpAnimationComplete){
      m_candle.animate(new SingleFadeAnimation(255, 30, 0, 0, 0.5, LedCount));
    }

    m_candle.modulateVBatOutput(1);
  }
}
