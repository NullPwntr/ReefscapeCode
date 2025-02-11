// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;

public class Coral extends SubsystemBase {

  public TalonFX coralIntake = new TalonFX(RobotConstants.CoralSubsystem.IntakeMotorId);

  XboxController operatorController = new XboxController(RobotConstants.Controllers.OperatorPortId);
  CommandXboxController commandOperatorController =
      new CommandXboxController(RobotConstants.Controllers.OperatorPortId);

  public Coral() {
    SmartDashboard.putNumber("CoralSpeed", 0.15);

    var Config = new MotorOutputConfigs();
    Config.NeutralMode = RobotConstants.CoralSubsystem.NeutralMode;

    coralIntake.getConfigurator().apply(Config);
  }

  @Override
  public void periodic() {
    // if (operatorController.getBButton()) {
    //   coralIntake.set(SmartDashboard.getNumber("CoralSpeed", 0.15));
    // } else if (operatorController.getYButton()) {
    //   coralIntake.set(-SmartDashboard.getNumber("CoralSpeed", 0.15));
    // } else {
    //   coralIntake.set(0);
    // }
  }
}
