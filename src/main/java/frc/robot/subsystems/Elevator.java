// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class Elevator extends SubsystemBase {

  TalonFX TopMotor = new TalonFX(RobotConstants.ElevatorSubsystem.TopMotorId);
  TalonFX BottomMotor = new TalonFX(RobotConstants.ElevatorSubsystem.BottomMotorId);

  private final PIDController pid =
      new PIDController(
          RobotConstants.ElevatorSubsystem.PIDFF.kP,
          RobotConstants.ElevatorSubsystem.PIDFF.kI,
          RobotConstants.ElevatorSubsystem.PIDFF.kD);
  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          RobotConstants.ElevatorSubsystem.PIDFF.kS,
          RobotConstants.ElevatorSubsystem.PIDFF.kG,
          RobotConstants.ElevatorSubsystem.PIDFF.kV,
          RobotConstants.ElevatorSubsystem.PIDFF.kA); // kS, kG, kV, kA

  XboxController operaController = new XboxController(RobotConstants.Controllers.OperatorPortId);

  /** Creates a new Elevator. */
  public Elevator() {
    var Config = new MotorOutputConfigs();
    // Config.Inverted = InvertedValue.CounterClockwise_Positive;
    Config.NeutralMode = RobotConstants.ElevatorSubsystem.NeutralMode;

    TopMotor.getConfigurator().apply(Config);
    BottomMotor.getConfigurator().apply(Config);

    TopMotor.getConfigurator().setPosition(0);
    BottomMotor.getConfigurator()
        .setPosition(
            0); // getSelectedSensorPosition(); (v5) => getPosition().getValueAsDouble(); (v6)

    pid.setSetpoint(0);

    // SmartDashboard.putNumber("Elevator_RightPos", 0);
    // SmartDashboard.putNumber("LeftPos", 0);
  }

  public void setElevatorVoltage(double Voltage) {
    TopMotor.setVoltage(Voltage);
    BottomMotor.setVoltage(Voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (operaController.getRightBumperButton()) {
      // TopMotor.set(RobotConstants.ElevatorSubsystem.MotorSpeed);
      // BottomMotor.set(RobotConstants.ElevatorSubsystem.MotorSpeed);
      pid.setSetpoint(20);
    } else if (operaController.getLeftBumperButton()) {
      // TopMotor.set(-RobotConstants.ElevatorSubsystem.MotorSpeed);
      // BottomMotor.set(-RobotConstants.ElevatorSubsystem.MotorSpeed);
      pid.setSetpoint(3);
    } else {
      // TopMotor.set(0);
      // BottomMotor.set(0);
      pid.setSetpoint(4);
    }

    setElevatorVoltage(pid.calculate(TopMotor.getPosition().getValueAsDouble()) * 12 * 0.5);

    SmartDashboard.putNumber("Elevator_RightPos", TopMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator_LeftPos", BottomMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("pid", pid.calculate(TopMotor.getPosition().getValueAsDouble()) * 12);
  }
}
