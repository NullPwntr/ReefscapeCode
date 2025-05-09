// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Motors
  TalonFX TopMotor = new TalonFX(RobotConstants.ElevatorSubsystem.TopMotorId, "CAN");
  TalonFX BottomMotor = new TalonFX(RobotConstants.ElevatorSubsystem.BottomMotorId, "CAN");

  // Sensors
  CANcoder elevatorCANCoder =
      new CANcoder(RobotConstants.ElevatorSubsystem.ElevatorCANCoderId, "CAN");

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

  // Variables
  @AutoLogOutput(key = "Elevator/Setpoint")
  double elevatorSetpoint = 0.0;

  @AutoLogOutput(key = "Elevator/Motion/PID_Output_RAW")
  double pidOutput = 0.0;

  @AutoLogOutput(key = "Elevator/Motion/FF_Output_RAW")
  double ffOutput = 0.0;

  double ascendSpeed = RobotConstants.ElevatorSubsystem.AscendMaxSpeed;
  double descendSpeed = RobotConstants.ElevatorSubsystem.DescendMaxSpeed;

  /** Creates a new Elevator. */
  public Elevator() {
    // Motor Config //
    TopMotor.getConfigurator().apply(RobotConstants.ElevatorSubsystem.Config.MotorsConfig);
    BottomMotor.getConfigurator().apply(RobotConstants.ElevatorSubsystem.Config.MotorsConfig);

    // Current Limiter //
    TopMotor.getConfigurator().apply(RobotConstants.ElevatorSubsystem.Config.CurrentConfig);
    BottomMotor.getConfigurator().apply(RobotConstants.ElevatorSubsystem.Config.CurrentConfig);
    //  //  //  //  //  //

    // Zeroing the selected sensor position
    // (Robot must always turn on when the whole elevator is down)
    TopMotor.getConfigurator().setPosition(0.0);
    BottomMotor.getConfigurator().setPosition(0.0);

    pid.setSetpoint(RobotConstants.ElevatorSubsystem.Setpoints.MinimumHeight);

    // SmartDashboard.putNumber("ElevkG", 0);
  }

  /** Sets the voltage of both of the elevator motors */
  public void setElevatorVoltage(double Voltage) {
    TopMotor.setVoltage(Voltage);
    BottomMotor.setVoltage(Voltage);
  }

  /** Sets the setpoint of the Elevator PID */
  public void setSetpoint(double setpoint) {
    elevatorSetpoint = setpoint;
  }

  /** Returns the average voltage from both of the elevator motors */
  @AutoLogOutput(key = "Elevator/AverageMotorVolts")
  public double getElevatorAverageVoltage() {
    return (TopMotor.getMotorVoltage().getValueAsDouble()
            + BottomMotor.getMotorVoltage().getValueAsDouble())
        / 2.0;
  }

  /** Returns the average voltage from both of the elevator motors */
  @AutoLogOutput(key = "Elevator/CANCoder Position")
  public double getElevatorCANCoderPosition() {
    return (elevatorCANCoder.getPosition().getValueAsDouble() + 0.119140625)
        * 9.234138486; // offset
  }

  @AutoLogOutput(key = "Elevator/CANCoder PositionRAW")
  public double getElevatorCANCoderPositionRAW() {
    return elevatorCANCoder.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    pid.setSetpoint(elevatorSetpoint);
    pidOutput = pid.calculate(getElevatorCANCoderPosition());
    ffOutput = feedforward.calculate(TopMotor.getVelocity().getValueAsDouble());

    if (DriverStation.isAutonomous()) {
      ascendSpeed = RobotConstants.ElevatorSubsystem.AscendMaxSpeed;
      descendSpeed = -0.6;
    } else {
      ascendSpeed = RobotConstants.ElevatorSubsystem.AscendMaxSpeed;
      descendSpeed = RobotConstants.ElevatorSubsystem.DescendMaxSpeed;
    }

    setElevatorVoltage(MathUtil.clamp(pidOutput, descendSpeed, ascendSpeed) * 12.0 + ffOutput);
    // setElevatorVoltage(SmartDashboard.getNumber("ElevkG", 0));

    // Advantage Scope Logging
    Logger.recordOutput("Elevator/TopMotor/Position", TopMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Elevator/TopMotor/Voltage", TopMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/BottomMotor/Position", BottomMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/BottomMotor/Voltage", TopMotor.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Elevator/Motion/PID Output (Volts)", pidOutput * 12.0);

    Logger.recordOutput("Elevator/Motion/PID Output", pidOutput);
    Logger.recordOutput("Elevator/Motion/FF Output", ffOutput);
  }
}
