package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Algae;

public class AlgaeCommands {

  private AlgaeCommands() {}

  public static Command Intake(Algae algae) {
    return Commands.runOnce(
        () -> {
          // Only intake when the robot doesn't have an algae.
          if (algae.hasAlgae() == false) {
            algae.algaeIntake.set(RobotConstants.AlgaeSubsystem.IntakeSpeed);
          }
        },
        algae);
  }

  public static Command IntakeCustomSpeed(Algae algae, double speed) {
    return Commands.runOnce(
        () -> {
          // Only intake when the robot doesn't have an algae.
          if (algae.hasAlgae() == false) {
            algae.algaeIntake.set(speed);
          }
        },
        algae);
  }

  public static Command Outtake(Algae algae) {
    return Commands.runOnce(
        () -> {
          algae.algaeIntake.set(RobotConstants.AlgaeSubsystem.OuttakeSpeed);
        },
        algae);
  }

  public static Command stopMotor(Algae algae) {
    return Commands.runOnce(
        () -> {
          algae.algaeIntake.set(0);
        },
        algae);
  }

  public static Command setSecondarySetpoint(Algae algae, double setpoint) {
    return Commands.runOnce(
        () -> {
          algae.setSecondaryArmSetpoint(setpoint);
        });
  }

  public static Command SetIsRunningCommand(Algae algae, boolean flag) {
    return Commands.runOnce(
        () -> {
          algae.setIsRunningCommand(flag);
        },
        algae);
  }

  public static Command SetIsLBHeld(Algae algae, boolean flag) {
    return Commands.runOnce(
        () -> {
          algae.setIsLBHeld(flag);
        },
        algae);
  }

  public static Command SetIsNetScoring(Algae algae, boolean flag) {
    return Commands.runOnce(
        () -> {
          algae.setIsNetScoring(flag);
        },
        algae);
  }
}
