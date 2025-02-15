// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Algae extends SubsystemBase {

  public TalonFX algaeIntake = new TalonFX(RobotConstants.AlgaeSubsystem.IntakeMotorId);

  public CANrange sensor = new CANrange(RobotConstants.AlgaeSubsystem.CANRangeId);

  XboxController operatorController = new XboxController(RobotConstants.Controllers.OperatorPortId);

  public Algae() {
    SmartDashboard.putNumber("AlgaeSpeed", 0.3);

    var Config = new MotorOutputConfigs();
    Config.NeutralMode = RobotConstants.AlgaeSubsystem.Config.NeutralMode;

    algaeIntake.getConfigurator().apply(Config);
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
    // if (operatorController.getBButton()) {
    //   algaeIntake.set(SmartDashboard.getNumber("AlgaeSpeed", 0.3));
    // } else if (operatorController.getYButton()) {
    //   algaeIntake.set(-0.5);
    // } else {
    //   algaeIntake.set(0.08);
    // }
  }
}
