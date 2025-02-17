package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Coral;

public class CoralCommands {

  private CoralCommands() {}

  public static Command Intake(Coral coral) {
    return Commands.run(
        () -> {
          // Only intake when the robot doesn't have a coral.
          if (coral.hasCoral() == false) {
            coral.coralIntake.set(RobotConstants.CoralSubsystem.IntakeSpeed);
          }
        },
        coral);
  }

  public static Command Outtake(Coral coral) {
    return Commands.run(
        () -> {
          coral.coralIntake.set(RobotConstants.CoralSubsystem.OuttakeSpeed);
        },
        coral);
  }

  public static Command OuttakeSlow(Coral coral) {
    return Commands.run(
        () -> {
          coral.coralIntake.set(-0.25);
        },
        coral);
  }

  public static Command stopMotor(Coral coral) {
    return Commands.runOnce(
        () -> {
          coral.coralIntake.set(0);
        },
        coral);
  }

  public static Command setAngle(Coral coral, double angle) {
    return Commands.runOnce(() -> {});
  }

  public static Command SetIsRunningCommand(Coral coral, boolean flag) {
    return Commands.runOnce(
        () -> {
          coral.setIsRunningCommand(flag);
        },
        coral);
  }
}
