// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {

  public TalonFX coralIntake = new TalonFX(RobotConstants.CoralSubsystem.IntakeMotorId);
  public TalonFX coralAngleMotor = new TalonFX(RobotConstants.CoralSubsystem.AngleSystem.MotorId);

  XboxController operatorController = new XboxController(RobotConstants.Controllers.OperatorPortId);
  CommandXboxController commandOperatorController =
      new CommandXboxController(RobotConstants.Controllers.OperatorPortId);

  private final PIDController pid =
      new PIDController(
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kP,
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kI,
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kD);

  @AutoLogOutput(key = "Coral/AngePIDOutput")
  double output = 0.0;

  @AutoLogOutput(key = "Coral/AngePIDSetpoint")
  double setpoint = 0.0;

  public Coral() {
    SmartDashboard.putNumber("CoralSpeed", 0.15);

    var Config = new MotorOutputConfigs();
    Config.NeutralMode = RobotConstants.CoralSubsystem.Config.NeutralMode;

    coralIntake.getConfigurator().apply(Config);
    coralAngleMotor.getConfigurator().apply(Config);

    coralAngleMotor.getConfigurator().setPosition(0.0);

    pid.setSetpoint(0);
    // pid.setTolerance(0.1);
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

    output = pid.calculate(coralAngleMotor.getPosition().getValueAsDouble());

    if (operatorController.getAButton()) {
      setpoint = 23;
    } else {
      setpoint = 1;
    }

    pid.setSetpoint(setpoint);

    coralAngleMotor.setVoltage(MathUtil.clamp(output, -0.1, 0.4) * 12.0);

    Logger.recordOutput(
        "Coral/CurrentAnglePosition", coralAngleMotor.getPosition().getValueAsDouble());

    // SmartDashboard.putNumber(
    //     "coralAnglePosition", coralAngleMotor.getPosition().getValueAsDouble());
  }
}
