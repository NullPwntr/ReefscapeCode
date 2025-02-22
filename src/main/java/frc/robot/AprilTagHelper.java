package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class AprilTagHelper {
  AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // AprilTag IDs for each alliance
  private final List<Integer> redAllianceReefTagIds = List.of(6, 7, 8, 9, 10, 11);
  private final List<Integer> blueAllianceReefTagIds = List.of(17, 18, 19, 20, 21, 22);

  private final double REEF_OFFSET = 0.5; // Adjust this distance for coral reef offset

  public Pose2d getReefRight(int aprilTagId) {
    Pose3d tagPose =
        field
            .getTagPose(aprilTagId)
            .orElseThrow(() -> new IllegalArgumentException("Invalid AprilTag ID: " + aprilTagId));

    double tagX = tagPose.getX();
    double tagY = tagPose.getY();
    double tagYaw = tagPose.getRotation().getZ(); // Rotation in radians

    // Compute right reef position (90 degrees from tagYaw)
    double reefX = tagX + REEF_OFFSET * Math.cos(tagYaw + Math.PI / 2);
    double reefY = tagY + REEF_OFFSET * Math.sin(tagYaw + Math.PI / 2);

    return new Pose2d(reefX, reefY, new Rotation2d(tagYaw));
  }

  public Pose2d getReefLeft(int aprilTagId) {
    Pose3d tagPose =
        field
            .getTagPose(aprilTagId)
            .orElseThrow(() -> new IllegalArgumentException("Invalid AprilTag ID: " + aprilTagId));

    double tagX = tagPose.getX();
    double tagY = tagPose.getY();
    double tagYaw = tagPose.getRotation().getZ(); // Rotation in radians

    // Compute left reef position (-90 degrees from tagYaw)
    double reefX = tagX + REEF_OFFSET * Math.cos(tagYaw - Math.PI / 2);
    double reefY = tagY + REEF_OFFSET * Math.sin(tagYaw - Math.PI / 2);

    return new Pose2d(reefX, reefY, new Rotation2d(tagYaw));
  }

  public int getClosestReefAprilTagToRobot(Pose2d robotPose, boolean isRedAlliance) {
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();

    List<Integer> validTagIds = isRedAlliance ? redAllianceReefTagIds : blueAllianceReefTagIds;

    int closestTagId = -1; // Default value if no tag is found
    double minDistance = Double.MAX_VALUE;

    for (int tagId : validTagIds) {
      Pose3d tagPose =
          field
              .getTagPose(tagId)
              .orElseThrow(() -> new IllegalArgumentException("Invalid AprilTag ID: " + tagId));

      double tagX = tagPose.getX();
      double tagY = tagPose.getY();

      // Calculate Euclidean distance
      double distance = Math.hypot(tagX - robotX, tagY - robotY);

      if (distance < minDistance) {
        minDistance = distance;
        closestTagId = tagId;
      }
    }

    if (closestTagId == -1) {
      throw new IllegalStateException("No valid AprilTag found for the given alliance.");
    }

    return closestTagId;
  }
}
