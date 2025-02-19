package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LEDs;

public class LEDCommands {

  private LEDCommands() {}

  public static Command SetColor(LEDs led, String color) {
    return Commands.runOnce(
        () -> {
          led.setColor(color);
        },
        led);
  }
  public static Command stayAtColor(LEDs led, String color) {
    return Commands.run(
        () -> {
          led.setColor(color);
        },
        led);
  }

  public static Command setIsRunningCommand(LEDs led, boolean flag) {
    return Commands.run(
        () -> {
          led.setIsRunningCommand(flag);
        },
        led);
  }

}
