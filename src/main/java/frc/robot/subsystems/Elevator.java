// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Motors
  TalonFX TopMotor = new TalonFX(RobotConstants.ElevatorSubsystem.TopMotorId);
  TalonFX BottomMotor = new TalonFX(RobotConstants.ElevatorSubsystem.BottomMotorId);

  // Motion Controls
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

  // Controllers
  XboxController operatorController = new XboxController(RobotConstants.Controllers.OperatorPortId);

  // Variables
  @AutoLogOutput(key = "Elevator/Setpoint")
  double elevatorSetpoint = 0.0;

  @AutoLogOutput(key = "Elevator/Motion/PID_Output_RAW")
  double pidOutput = 0.0;

  @AutoLogOutput(key = "Elevator/Motion/FF_Output_RAW")
  double ffOutput = 0.0;

  /** Creates a new Elevator. */
  public Elevator() {
    // Motor Config //
    var Config = new MotorOutputConfigs();
    Config.Inverted = RobotConstants.ElevatorSubsystem.Config.MotorInverted;
    Config.NeutralMode = RobotConstants.ElevatorSubsystem.Config.NeutralMode;

    TopMotor.getConfigurator().apply(Config);
    BottomMotor.getConfigurator().apply(Config);

    // Current Limiter //
    var currentLimits = new CurrentLimitsConfigs();

    currentLimits.StatorCurrentLimit = RobotConstants.ElevatorSubsystem.Config.CurrentLimit;
    currentLimits.StatorCurrentLimitEnable = false; // Enable stator current limiting

    TopMotor.getConfigurator().apply(currentLimits);
    BottomMotor.getConfigurator().apply(currentLimits);
    //  //  //  //  //  //

    // Zeroing the selected sensor position
    // (Robot must always turn on when the whole elevator is down)
    TopMotor.getConfigurator().setPosition(0.0);
    BottomMotor.getConfigurator().setPosition(0.0);

    pid.setSetpoint(0);

    SmartDashboard.putNumber(
        "ElevatorCustom_kP", RobotConstants.ElevatorSubsystem.PIDFF.kP); // temporary
    SmartDashboard.putNumber("ElevatorCustom_Setpoint", 0.0); // temporary
  }

  /** Sets the voltage of both of the elevator motors */
  public void setElevatorVoltage(double Voltage) {
    TopMotor.setVoltage(Voltage);
    BottomMotor.setVoltage(Voltage);
  }

  /** Sets the setpoint of the Elevator PID */
  // public void setSetpoint(double setpoint) {
  //   pid.setSetpoint(setpoint);
  // }

  /** Returns the average voltage from both of the elevator motors */
  @AutoLogOutput(key = "Elevator/AverageMotorVolts")
  public double getElevatorAverageVoltage() {
    return (TopMotor.getMotorVoltage().getValueAsDouble()
            + BottomMotor.getMotorVoltage().getValueAsDouble())
        / 2.0;
  }

  @Override
  public void periodic() {
    pid.setP(
        SmartDashboard.getNumber(
            "ElevatorCustom_kP", RobotConstants.ElevatorSubsystem.PIDFF.kP)); // temporary

    if (operatorController.getRightBumperButton()) {
      // TopMotor.set(RobotConstants.ElevatorSubsystem.MotorSpeed);
      // BottomMotor.set(RobotConstants.ElevatorSubsystem.MotorSpeed);
      elevatorSetpoint = 30.0;
    } else if (operatorController.getLeftBumperButton()) {
      // TopMotor.set(-RobotConstants.ElevatorSubsystem.MotorSpeed);
      // BottomMotor.set(-RobotConstants.ElevatorSubsystem.MotorSpeed);
      elevatorSetpoint = 10; // SmartDashboard.getNumber("ElevatorCustom_Setpoint", 0); // temporary
    } else {
      // TopMotor.set(0);
      // BottomMotor.set(0);
      elevatorSetpoint = 4.0;
    }

    pid.setSetpoint(elevatorSetpoint);
    pidOutput = pid.calculate(TopMotor.getPosition().getValueAsDouble());
    ffOutput = feedforward.calculate(TopMotor.getVelocity().getValueAsDouble());

    setElevatorVoltage(
        MathUtil.clamp(pidOutput, -0.1, 0.3) * 12.0 // 0.5
            + feedforward.calculate(TopMotor.getVelocity().getValueAsDouble()));
    // setElevatorVoltage(0);

    // Advantage Scope Logging
    Logger.recordOutput("Elevator/TopMotor/Position", TopMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Elevator/TopMotor/Voltage", TopMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/BottomMotor/Position", BottomMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/BottomMotor/Voltage", TopMotor.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput(
        "Elevator/Motion/PID Output (Volts)",
        pid.calculate(TopMotor.getPosition().getValueAsDouble()) * 12.0);

    Logger.recordOutput("Elevator/Motion/PID Output", pidOutput);
  }
}
