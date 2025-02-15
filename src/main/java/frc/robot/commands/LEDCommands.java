package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LEDs;

public class LEDCommands {

  private LEDCommands() {}

  public static Command setState(LEDs led, String state) {
    return Commands.runOnce(
        () -> {
          led.setState(state);
        },
        led);
  }

  //   public static Command runBootUpAnimation(LEDs led) {
  //     return Commands.runOnce(
  //         () -> {
  //          // Complete this
  //         },
  //         led);
  //   }
}
