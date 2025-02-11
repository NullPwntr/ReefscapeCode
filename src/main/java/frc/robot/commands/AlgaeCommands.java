package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Algae;

public class AlgaeCommands {

  private AlgaeCommands() {}

  public static Command Intake(Algae algae) {
    return Commands.run(
        () -> {
          algae.algaeIntake.set(RobotConstants.AlgaeSubsystem.IntakeSpeed);
        },
        algae);
  }

  public static Command Outtake(Algae algae) {
    return Commands.run(
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

  // public static Command setAngle(Coral coral, double angle) {
  //   return Commands.runOnce(
  //           () -> {
  //             // do this
  //           });
  // }

}
