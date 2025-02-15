// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {

  public TalonFX coralIntake = new TalonFX(RobotConstants.CoralSubsystem.IntakeMotorId);
  public TalonFX coralAngleMotor = new TalonFX(RobotConstants.CoralSubsystem.AngleSystem.MotorId);
  private final double GEAR_RATIO = 45.0; // 1:45 gear ratio
  private final double TICKS_PER_REV = 2048.0; // TalonFX Encoder Resolution

  public CANrange sensor = new CANrange(RobotConstants.CoralSubsystem.CANRangeId);

  XboxController operatorController = new XboxController(RobotConstants.Controllers.OperatorPortId);
  CommandXboxController commandOperatorController =
      new CommandXboxController(RobotConstants.Controllers.OperatorPortId);

  private final PIDController pid =
      new PIDController(
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kP,
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kI,
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kD);

  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kS,
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kG,
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kV,
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kA); // kS, kG, kV, kA

  @AutoLogOutput(key = "Coral/AngePIDOutput")
  double output = 0.0;

  @AutoLogOutput(key = "Coral/AngePIDSetpoint")
  double setpoint = 0.0;

  boolean isRunningCommand = false;

  public Coral() {
    // Motor Config //
    var Config = new MotorOutputConfigs();
    Config.NeutralMode = RobotConstants.CoralSubsystem.Config.NeutralMode;
    Config.Inverted = RobotConstants.CoralSubsystem.Config.MotorInverted;
    var AngleConfig = new MotorOutputConfigs();
    AngleConfig.NeutralMode = RobotConstants.CoralSubsystem.Config.NeutralMode;
    AngleConfig.Inverted = RobotConstants.CoralSubsystem.Config.AngleMotorInverted;

    coralIntake.getConfigurator().apply(Config);
    coralAngleMotor.getConfigurator().apply(AngleConfig);

    // Current Limiter //
    var currentLimits = new CurrentLimitsConfigs();

    currentLimits.StatorCurrentLimit = RobotConstants.CoralSubsystem.Config.CurrentLimit;
    currentLimits.StatorCurrentLimitEnable = true; // Enable stator current limiting

    coralAngleMotor.getConfigurator().apply(currentLimits);
    //  //  //  //  //  //

    coralAngleMotor.getConfigurator().setPosition(0.0);

    pid.setSetpoint(0);
  }

  @AutoLogOutput(key = "Coral/Sensor/Distance")
  public double getSensorDistance() {
    return sensor.getDistance().getValueAsDouble();
  }

  @AutoLogOutput(key = "Coral/Angle/Rotations")
  public double getOutputRotations() {
    double motorPosition = coralAngleMotor.getPosition().getValueAsDouble(); // Get encoder ticks
    return motorPosition / (GEAR_RATIO * TICKS_PER_REV); // Convert back to output rotations
  }

  @AutoLogOutput(key = "Coral/HasCoral")
  public boolean hasCoral() {
    return (sensor.getDistance().getValueAsDouble()
                <= RobotConstants.CoralSubsystem.hasCoralThreshold
            && sensor.getAmbientSignal().getValueAsDouble() <= 10)
        ? true
        : false;
  }

  public void setSetpoint(double Setpoint) {
    setpoint = Setpoint;
  }

  public void setIsRunningCommand(boolean flag) {
    isRunningCommand = flag;
  }

  @Override
  public void periodic() {
    output = pid.calculate(coralAngleMotor.getPosition().getValueAsDouble());

    // if (operatorController.getAButton()) {
    //   setpoint = RobotConstants.CoralSubsystem.Setpoints.NormalScoring;
    // } else {
    //   setpoint = RobotConstants.CoralSubsystem.Setpoints.Center;
    // }

    if (hasCoral()) {
      if (isRunningCommand == false) {
        setpoint = RobotConstants.CoralSubsystem.Setpoints.Center;
      }
    } else {
      setpoint = RobotConstants.CoralSubsystem.Setpoints.HumanIntake;
    }

    pid.setSetpoint(setpoint);

    coralAngleMotor.setVoltage(
        MathUtil.clamp(
                    output,
                    RobotConstants.CoralSubsystem.AngleSystem.ReturnMaxSpeed,
                    RobotConstants.CoralSubsystem.AngleSystem.LaunchMaxSpeed)
                * 12.0
            + feedforward.calculate(setpoint, output));

    Logger.recordOutput(
        "Coral/CurrentAnglePosition", coralAngleMotor.getPosition().getValueAsDouble());
  }
}
