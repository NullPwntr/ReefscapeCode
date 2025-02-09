// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {

  TalonFX coralIntake = new TalonFX(31);

  XboxController operatorController = new XboxController(1);

  /** Creates a new Coral. */
  public Algae() {
    SmartDashboard.putNumber("AlgaeSpeed", 0.3);

    var breakConfig = new MotorOutputConfigs();
    breakConfig.NeutralMode = NeutralModeValue.Brake;

    coralIntake.getConfigurator().apply(breakConfig);
  }

  @Override
  public void periodic() {
    // if (operatorController.getBButton()) {
    //   coralIntake.set(SmartDashboard.getNumber("AlgaeSpeed", 0.3));
    // } else if (operatorController.getYButton()) {
    //   coralIntake.set(-0.5);
    // } else {
    //   coralIntake.set(0.08);
    // }
  }
}
