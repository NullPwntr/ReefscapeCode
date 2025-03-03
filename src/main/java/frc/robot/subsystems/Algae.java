// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  // Motors
  public TalonFX algaeIntake = new TalonFX(RobotConstants.AlgaeSubsystem.IntakeMotorId);
  public TalonFX SecondaryArm = new TalonFX(RobotConstants.AlgaeSubsystem.SecondaryArmMotorId);

  // Sensors
  public CANcoder secondaryArmCANCoder =
      new CANcoder(RobotConstants.AlgaeSubsystem.SecondaryArmCANCoderId);

  public CANrange sensor = new CANrange(RobotConstants.AlgaeSubsystem.CANRangeId);

  // Motion Controls
  public PIDController secondaryPID =
      new PIDController(
          RobotConstants.AlgaeSubsystem.SecondaryArm.PIDFF.kP,
          RobotConstants.AlgaeSubsystem.SecondaryArm.PIDFF.kI,
          RobotConstants.AlgaeSubsystem.SecondaryArm.PIDFF.kD);

  // Variables
  @AutoLogOutput(key = "Algae/PIDOutput")
  double output = 0.0;

  @AutoLogOutput(key = "Algae/IsRunningCommand")
  boolean isRunningCommand = false;

  boolean LBHeld = false;
  boolean isNetScoring = false;

  public Algae() {
    // Motor/CANCoder Config //
    algaeIntake.getConfigurator().apply(RobotConstants.AlgaeSubsystem.Config.IntakeConfig);
    SecondaryArm.getConfigurator().apply(RobotConstants.AlgaeSubsystem.Config.SecondaryConfig);

    secondaryArmCANCoder
        .getConfigurator()
        .apply(RobotConstants.AlgaeSubsystem.Config.CANCoderConfig);
    //  //  //  //  //  //

    SecondaryArm.getConfigurator().setPosition(0.0);
    secondaryPID.setSetpoint(0);

    // SmartDashboard.putNumber("DEBUG_algae_kp", 0);
    // SmartDashboard.putNumber("DEBUG_algae_ki", 0);
    // SmartDashboard.putNumber("DEBUG_algae_kd", 0);
  }

  /** Returns the current sensor distance */
  @AutoLogOutput(key = "Algae/Sensor/Distance")
  public double getSensorDistance() {
    return sensor.getDistance().getValueAsDouble();
  }

  /** Returns true if the algae subsystem senses an algae, reutrns false otherwise */
  @AutoLogOutput(key = "Algae/HasAlgae")
  public boolean hasAlgae() {
    return (sensor.getDistance().getValueAsDouble()
            <= RobotConstants.AlgaeSubsystem.hasAlgaeThreshold
        && sensor.getAmbientSignal().getValueAsDouble() <= 10);
  }

  /** Changes the algae arm setpoint */
  public void setSecondaryArmSetpoint(double setpoint) {
    secondaryPID.setSetpoint(setpoint);
  }

  /** Returns the current algae arm angle position (CANCoder [0-100]) */
  @AutoLogOutput(key = "Algae/Secondary Arm/Position")
  public double getSecondaryArmPosition() {
    return (secondaryArmCANCoder.getPosition().getValueAsDouble()) * 100; // offset
  }

  /** Updates the LED state based on the sensors */
  public void updateLEDs() {
    if (hasAlgae()) {
      LEDs.currentColor = "CYAN";
    } else {
      if (LEDs.currentColor != "WHITE") { // idk how this works but it does
        LEDs.currentColor = "DEFAULT";
      }
    }
  }

  // FLAG METHODS //
  /**
   * Method that indicates to the subsystem that there is a command currently using the current
   * subsystem
   */
  public void setIsRunningCommand(boolean flag) {
    isRunningCommand = flag;
  }
  /**
   * Method that indicates to the subsystem that the operator is holding the LB button (algae
   * outtake)
   */
  public void setIsLBHeld(boolean flag) {
    LBHeld = flag;
  }
  /** Method that indicates to the subsystem that the operator is algae net scoring */
  public void setIsNetScoring(boolean flag) {
    isNetScoring = flag;
  }

  public boolean isNetScoring() {
    return isNetScoring;
  }
  // // // // // // //

  @Override
  public void periodic() {
    output = secondaryPID.calculate(getSecondaryArmPosition());

    if (hasAlgae() && LBHeld == false && isNetScoring == false) {
      isRunningCommand = false;
    }

    if (hasAlgae() && isRunningCommand == false) {
      setSecondaryArmSetpoint(0);
      algaeIntake.set(0.06);
    }

    SecondaryArm.setVoltage(output * 12.0);

    updateLEDs();

    Logger.recordOutput("Algae/Motion/SecondarySetpoint", secondaryPID.getSetpoint());
    Logger.recordOutput("Algae/Motion/SecondaryOutput", output);
  }
}
