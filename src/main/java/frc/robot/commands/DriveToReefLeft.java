package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveToReefLeft extends Command {
  private final Drive drive;
  private Command pathfindingCommand;

  public DriveToReefLeft(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    updatePathCommand(); // Generate path when command starts
  }

  @Override
  public void execute() {
    // Continuously update the path if necessary (e.g., if robot position changes significantly)
    updatePathCommand();
  }

  private void updatePathCommand() {
    PathConstraints constraints =
        new PathConstraints(2, 2, Math.toRadians(180), Math.toRadians(180));

    // Get the closest reef position dynamically
    Pose2d targetPose = drive.getClosesPose2dLeft();

    // Create and schedule a new path command
    if (pathfindingCommand != null) {
      pathfindingCommand.cancel(); // Stop the previous path if necessary
    }
    pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
    pathfindingCommand.schedule();
    System.out.println("🔄 Updating path to reef: " + targetPose);
  }

  @Override
  public boolean isFinished() {
    return pathfindingCommand != null && pathfindingCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("❌ DriveToReefLeft INTERRUPTED!");
    } else {
      System.out.println("✅ DriveToReefLeft COMPLETED!");
    }
    drive.runVelocity(new ChassisSpeeds()); // Stop the drive
  }
}
