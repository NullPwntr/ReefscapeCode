// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class Climb extends SubsystemBase {
  XboxController operatorController = new XboxController(RobotConstants.Controllers.OperatorPortId);
  TalonFX climber = new TalonFX(13, "CAN");

  double climbingSpeed = 1;

  /** Creates a new Climb. */
  public Climb() {
    climber
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));

    climber
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40.0)
                .withStatorCurrentLimitEnable(true));
  }

  @Override
  public void periodic() {
    if (operatorController.getRightTriggerAxis() > 0.1) {
      climber.set(climbingSpeed);
    } else if (operatorController.getLeftTriggerAxis() > 0.1) {
      climber.set(-climbingSpeed);
    } else {
      climber.set(0);
    }
  }
}
