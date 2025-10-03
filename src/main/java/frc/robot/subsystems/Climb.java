// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  XboxController operatorController = new XboxController(RobotConstants.Controllers.OperatorPortId);

  TalonFX claw = new TalonFX(19, "CAN");
  TalonFX climb = new TalonFX(13, "CAN");

  boolean Climbed = false;

  double clawSpeed = 0.3;

  public Climb() {}

  @Override
  public void periodic() {
    if (operatorController.getRightTriggerAxis() > 0.1) {
      claw.set(-clawSpeed);
    } else {
      claw.set(0);
    }

    if (Math.abs(operatorController.getRightY()) > 0.1) {
      double climbRotSpeed = Math.signum(operatorController.getRightY());
      climb.set(-climbRotSpeed);

    } else {
      climb.set(0);
    }

    // need a better electrical detection without using a switch
    // (if spike is positive, ?have a good ratio with the battery voltage?)
    if (claw.getSupplyCurrent().getValueAsDouble() >= 20) {
      Climbed = true;
    }

    Logger.recordOutput("Climber/Claw Voltage", claw.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Climber/Claw Supply Current", claw.getSupplyCurrent().getValueAsDouble());

    Logger.recordOutput("Climbed", Climbed);
  }
}
