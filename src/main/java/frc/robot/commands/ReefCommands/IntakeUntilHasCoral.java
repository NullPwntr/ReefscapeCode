package frc.robot.commands.ReefCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Coral;

public class IntakeUntilHasCoral extends Command {
  private final Coral coral;

  public IntakeUntilHasCoral(Coral coral) {
    this.coral = coral;

    addRequirements(coral);
  }

  @Override
  public void initialize() {
    coral.coralIntake.set(RobotConstants.CoralSubsystem.IntakeSpeed);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return coral.hasCoral();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("❌ IntakeUntilHasCoral Interrupted!");
    } else {
      System.out.println("✅ IntakeUntilHasCoral Completed Successfully!");
    }
    coral.coralIntake.set(0);
  }
}
