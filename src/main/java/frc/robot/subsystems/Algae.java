// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
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

  public CANcoder secondaryArmCANCoder =
      new CANcoder(RobotConstants.AlgaeSubsystem.SecondaryArmCANCoderId);

  public CANrange sensor = new CANrange(RobotConstants.AlgaeSubsystem.CANRangeId);

  public PIDController primaryPID = new PIDController(0.005, 0, 0);
  public PIDController secondaryPID =
      new PIDController(0.0033, 0, 0.000014); // wtf (near perfect gains)

  XboxController operatorController = new XboxController(RobotConstants.Controllers.OperatorPortId);
  XboxController debugController = new XboxController(RobotConstants.Controllers.DebugPortId);

  @AutoLogOutput(key = "Algae/IsRunningCommand")
  boolean isRunningCommand = false;

  boolean LBHeld = false;
  boolean isNetScoring = false;

  public Algae() {
    SmartDashboard.putNumber("AlgaeSpeed", 0.3);

    var Config = new MotorOutputConfigs();
    Config.NeutralMode = RobotConstants.AlgaeSubsystem.Config.NeutralMode;
    Config.Inverted = InvertedValue.CounterClockwise_Positive;
    var SecondaryConfig = new MotorOutputConfigs();
    SecondaryConfig.NeutralMode = RobotConstants.AlgaeSubsystem.Config.NeutralMode;
    SecondaryConfig.Inverted = InvertedValue.Clockwise_Positive;

    algaeIntake.getConfigurator().apply(Config);

    PrimaryArm.getConfigurator().apply(SecondaryConfig);
    SecondaryArm.getConfigurator().apply(SecondaryConfig);

    PrimaryArm.getConfigurator().setPosition(0.0);
    SecondaryArm.getConfigurator().setPosition(0.0);

    primaryPID.setSetpoint(0);
    secondaryPID.setSetpoint(0);

    CANcoderConfiguration CANConfig = new CANcoderConfiguration();
    CANConfig.MagnetSensor.SensorDirection =
        RobotConstants.AlgaeSubsystem.SecondaryArmCANCoderDirection;

    secondaryArmCANCoder.getConfigurator().apply(CANConfig);

    // SmartDashboard.putNumber("ttt", 0.005);
    // SmartDashboard.putNumber("tttsetpoint", 35.0);
    // SmartDashboard.putNumber("tttkp", 0);
    // SmartDashboard.putNumber("tttki", 0);
    // SmartDashboard.putNumber("tttkd", 0);
    // SmartDashboard.putNumber("tttizone", 0);
    // SmartDashboard.putNumber("tttkg", 0.56);
  }

  @AutoLogOutput(key = "Algae/Sensor/Distance")
  public double getSensorDistance() {
    return sensor.getDistance().getValueAsDouble();
  }

  @AutoLogOutput(key = "Algae/HasAlgae")
  public boolean hasAlgae() {
    return (sensor.getDistance().getValueAsDouble()
                <= RobotConstants.AlgaeSubsystem.hasAlgaeThreshold
            && sensor.getAmbientSignal().getValueAsDouble() <= 10)
        ? true
        : false;
  }

  public void setPrimaryArmSetpoint(double setpoint) {
    primaryPID.setSetpoint(setpoint);
  }

  public void setSecondaryArmSetpoint(double setpoint) {
    secondaryPID.setSetpoint(setpoint);
  }

  @AutoLogOutput(key = "Algae/Primary Arm/Position")
  public double getPrimaryArmPosition() {
    return (PrimaryArm.getPosition().getValueAsDouble()) * 10; // offset
  }

  @AutoLogOutput(key = "Algae/Secondary Arm/Position")
  public double getSecondaryArmPosition() {
    return (secondaryArmCANCoder.getPosition().getValueAsDouble() - 0.089111) * 100; // offset
  }

  public void setIsRunningCommand(boolean flag) {
    isRunningCommand = flag;
  }

  public void setIsLBHeld(boolean flag) {
    LBHeld = flag;
  }

  public void setIsNetScoring(boolean flag) {
    isNetScoring = flag;
  }

  public boolean isNetScoring() {
    return isNetScoring;
  }

  @Override
  public void periodic() {
    // if (operatorController.getAButton()) {
    //   // algaeIntake.set(SmartDashboard.getNumber("AlgaeSpeed", 0.3));
    //   // algaeIntake.set(0.5);
    //   // secondaryPID.setSetpoint(50);
    //   primaryPID.setSetpoint(90);
    //   secondaryPID.setSetpoint(70);
    // } else if (operatorController.getXButton()) {
    //   algaeIntake.set(-0.5);
    // } else {
    //   algaeIntake.set(0.06);
    //   secondaryPID.setSetpoint(0);
    //   primaryPID.setSetpoint(0);
    // }

    // // secondaryPID.setP(SmartDashboard.getNumber("ttt", 0.005));

    // secondaryPID.setIZone(SmartDashboard.getNumber("tttizone", 0.0));
    // secondaryPID.setSetpoint(SmartDashboard.getNumber("tttsetpoint", 35.0));

    if (hasAlgae()) {
      LEDs.currentColor = "CYAN";
    } else {
      if (LEDs.currentColor != "WHITE") { // idk how this works but it does
        LEDs.currentColor = "DEFAULT";
      }
    }

    if (hasAlgae() && LBHeld == false && isNetScoring == false) {
      isRunningCommand = false;
    }

    if (hasAlgae() && isRunningCommand == false) {
      setSecondaryArmSetpoint(0);
      algaeIntake.set(0.06);
    }

    // if (debugController.getLeftBumperButton()) {
    //   algaeIntake.set(-3.0);
    // } else {
    //   if (hasAlgae() && isRunningCommand == false) {
    //     setSecondaryArmSetpoint(0);
    //     algaeIntake.set(0.06);
    //   } else {
    //     algaeIntake.set(0.06);
    //   }
    // }

    // if (debugController.getLeftBumperButton()) {
    //   algaeIntake.set(-3.0);
    // } else {
    //   algaeIntake.set(0);
    // }

    PrimaryArm.setVoltage((primaryPID.calculate(getPrimaryArmPosition()) * 12.0));
    SecondaryArm.setVoltage((secondaryPID.calculate(getSecondaryArmPosition()) * 12.0));

    // SecondaryArm.set(SmartDashboard.getNumber("ttt", 0.0));

    SmartDashboard.putNumber("AMPARM", SecondaryArm.getSupplyCurrent().getValueAsDouble());
    // Logger.recordOutput("Algae/Primary Arm/Position",
    // PrimaryArm.getPosition().getValueAsDouble());
    // Logger.recordOutput(
    //     "Algae/Secondary Arm/Position", SecondaryArm.getPosition().getValueAsDouble());
    Logger.recordOutput("Algae/Motion/PrimarySetpoint", primaryPID.getSetpoint());
    Logger.recordOutput("Algae/Motion/SecondarySetpoint", secondaryPID.getSetpoint());
    Logger.recordOutput(
        "Algae/Motion/SecondaryOutput",
        secondaryPID.calculate(SecondaryArm.getPosition().getValueAsDouble()));
  }
}
