// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {

  public TalonFX algaeIntake = new TalonFX(RobotConstants.AlgaeSubsystem.IntakeMotorId);

  public TalonFX PrimaryArm = new TalonFX(RobotConstants.AlgaeSubsystem.PrimaryArmMotorId);
  public TalonFX SecondaryArm = new TalonFX(RobotConstants.AlgaeSubsystem.SecondaryArmMotorId);

  public CANrange sensor = new CANrange(RobotConstants.AlgaeSubsystem.CANRangeId);

  public PIDController primaryPID = new PIDController(0, 0, 0);
  public PIDController secondaryPID = new PIDController(4, 0, 0);

  XboxController operatorController = new XboxController(RobotConstants.Controllers.OperatorPortId);

  public Algae() {
    SmartDashboard.putNumber("AlgaeSpeed", 0.3);

    var Config = new MotorOutputConfigs();
    Config.NeutralMode = RobotConstants.AlgaeSubsystem.Config.NeutralMode;
    Config.Inverted = InvertedValue.CounterClockwise_Positive;

    algaeIntake.getConfigurator().apply(Config);

    PrimaryArm.getConfigurator().apply(Config);
    SecondaryArm.getConfigurator().apply(Config);

    PrimaryArm.getConfigurator().setPosition(0.0);
    SecondaryArm.getConfigurator().setPosition(0.0);

    primaryPID.setSetpoint(0);
    secondaryPID.setSetpoint(0);

    SmartDashboard.putNumber("ttt", 0.0);
  }

  @AutoLogOutput(key = "Algae/Sensor/Distance")
  public double getSensorDistance() {
    return sensor.getDistance().getValueAsDouble();
  }

  @AutoLogOutput(key = "Algae/HasAlgae")
  public boolean hasCoral() {
    return (sensor.getDistance().getValueAsDouble()
            <= RobotConstants.AlgaeSubsystem.hasAlgaeThreshold)
        ? true
        : false;
  }

  @Override
  public void periodic() {
    // if (operatorController.getAButton()) {
    //   // algaeIntake.set(SmartDashboard.getNumber("AlgaeSpeed", 0.3));
    //   algaeIntake.set(0.5);
    // } else if (operatorController.getXButton()) {
    //   algaeIntake.set(-0.5);
    // } else {
    //   algaeIntake.set(0.08);
    // }

    // secondaryPID.setSetpoint(0.5);

    // SecondaryArm.set(secondaryPID.calculate(SecondaryArm.getPosition().getValueAsDouble()));

    SecondaryArm.set(SmartDashboard.getNumber("ttt", 0.0));

    SmartDashboard.putNumber("AMPARM", SecondaryArm.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Algae/Primary Arm/Position", PrimaryArm.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "Algae/Secondary Arm/Position", SecondaryArm.getPosition().getValueAsDouble());
    Logger.recordOutput("Algae/Motion/PrimarySetpoint", primaryPID.getSetpoint());
    Logger.recordOutput("Algae/Motion/SecondarySetpoint", secondaryPID.getSetpoint());
    Logger.recordOutput(
        "Algae/Motion/SecondaryOutput",
        secondaryPID.calculate(SecondaryArm.getPosition().getValueAsDouble()));
  }
}
