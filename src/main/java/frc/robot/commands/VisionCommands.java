package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;

public class VisionCommands {

  private VisionCommands() {}

  public static Command TurnOffVisionPoseEstimation() {
    return Commands.runOnce(
        () -> {
          LimelightHelpers.setPipelineIndex("limelight-right", 1);
        });
  }

  public static Command TurnOnVisionPoseEstimation() {
    return Commands.runOnce(
        () -> {
          LimelightHelpers.setPipelineIndex("limelight-right", 0);
        });
  }
}
