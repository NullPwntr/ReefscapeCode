package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;

public class ElevatorCommands {

  private ElevatorCommands() {}

  public static Command SetSetpoint(Elevator elevator, double setpoint) {
    return Commands.run(
        () -> {
          elevator.setSetpoint(setpoint);
        },
        elevator);
  }
}
