// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  Pigeon2 gyro = new Pigeon2(8);

  // AprilTag IDs for each alliance
  private final List<Integer> redAllianceReefTagIds = List.of(6, 7, 8, 9, 10, 11);
  private final List<Integer> blueAllianceReefTagIds = List.of(17, 18, 19, 20, 21, 22);

  private final List<Integer> redAllianceHumanTagIds = List.of(1, 2);
  private final List<Integer> blueAllianceHumanTagIds = List.of(12, 13);

  // Define separate offsets for X (right-left) and Y (forward-backward)
  private final double REEF_X_OFFSET = 0.1775; // Moves perpendicular to tag orientation
  private final double REEF_Y_OFFSET = 0.8; // Moves parallel to tag orientation - 0.3

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // this.setPose(new Pose2d(this.getPose().getTranslation(), new Rotation2d()));
  }

  // public void autoGyroZeroWithVision() {
  //   var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  //   String tableKey = (alliance == Alliance.Red) ? "botpose_wpired" : "botpose_wpiblue";

  //   NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-right");
  //   double[] botPose = limelightTable.getEntry(tableKey).getDoubleArray(new double[6]);
  //   double tv = limelightTable.getEntry("tv").getDouble(0); // target valid
  //   double latency = limelightTable.getEntry("tl").getDouble(100); // vision latency

  //   // Only proceed if target is valid and pose looks good
  //   if (tv < 1 || botPose.length < 6 || latency > 30) {
  //     return; // No valid tag or data is stale
  //   }

  //   // Extract heading from Limelight pose
  //   double tagYaw = botPose[5]; // in degrees
  //   Rotation2d visionHeading = Rotation2d.fromDegrees(tagYaw);

  //   // Get current gyro heading
  //   Rotation2d gyroHeading =
  //       Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()); // adjust for your gyro
  //   Rotation2d difference = visionHeading.minus(gyroHeading);

  //   // Only zero if heading is off by enough to matter
  //   if (Math.abs(difference.getDegrees()) > 2.0) {
  //     gyro.setYaw(tagYaw + 180);
  //     System.out.println("[AutoGyroZero] Gyro reset to AprilTag heading: " + tagYaw);
  //   }
  // }

  public double zeroYawInitial() {
    double previousYaw = gyro.getYaw().getValueAsDouble();
    System.out.println("Old Yaw: " + previousYaw);
    if (DriverStation.getAlliance().get() != Alliance.Red) { // If Alliance is NOT RED
      // System.out.println("Yaw 180 " + RobotContainer.isAllianceRed);

      StatusCode status = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; ++i) {
        status = gyro.setYaw(180.0);
        if (status.isOK()) break;
      }
      if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
      }

      // Reset IMU pose; may need to remove for the competition
      // setCurrentOdometryPoseToSpecificRotation(180);

    } else {
      // System.out.println("Yaw NOT 180 " + RobotContainer.isAllianceRed);

      StatusCode status = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; ++i) {
        status = gyro.setYaw(0);
        if (status.isOK()) break;
      }
      if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
      }

      // setCurrentOdometryPoseToSpecificRotation(0);
    }
    System.out.println("New Yaw: " + gyro.getYaw());
    return previousYaw;
  }

  @Override
  public void periodic() {
    // autoGyroZeroWithVision();

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "Poses/PoseClosestLeft")
  public Pose2d getClosesPose2dLeft() {
    return getReefLeft(getClosestReefAprilTagToRobot());
  }

  @AutoLogOutput(key = "Poses/PoseClosestLeftClose")
  public Pose2d getClosesPose2dLeftClose() {
    return getReefLeftClose(getClosestReefAprilTagToRobot());
  }

  @AutoLogOutput(key = "Poses/PoseClosestRight")
  public Pose2d getClosesPose2dRight() {
    return getReefRight(getClosestReefAprilTagToRobot());
  }

  @AutoLogOutput(key = "Poses/PoseClosestRightClose")
  public Pose2d getClosesPose2dRightClose() {
    return getReefRightClose(getClosestReefAprilTagToRobot());
  }

  @AutoLogOutput(key = "Poses/PoseClosestCenter")
  public Pose2d getClosesPose2dCenter() {
    return getReefCenter(getClosestReefAprilTagToRobot());
  }

  @AutoLogOutput(key = "Poses/PoseClosestCenterClose")
  public Pose2d getClosesPose2dCenterClose() {
    return getReefCenterClose(getClosestReefAprilTagToRobot());
  }

  @AutoLogOutput(key = "Poses/PoseClosestHuman")
  public Pose2d getClosesPose2dHuman() {
    return getHumanPose2d(getClosestHumanAprilTagToRobot());
  }

  @AutoLogOutput(key = "Poses/AprilTagIdClosest")
  public int getClosestReefAprilTagToRobot() {
    double robotX = getPose().getX();
    double robotY = getPose().getY();

    List<Integer> validTagIds =
        (DriverStation.getAlliance().get() == Alliance.Red)
            ? redAllianceReefTagIds
            : blueAllianceReefTagIds;

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

  @AutoLogOutput(key = "Poses/AprilTagIdClosest")
  public int getClosestHumanAprilTagToRobot() {
    double robotX = getPose().getX();
    double robotY = getPose().getY();

    List<Integer> validTagIds =
        (DriverStation.getAlliance().get() == Alliance.Red)
            ? redAllianceHumanTagIds
            : blueAllianceHumanTagIds;

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

  public Pose2d getReefRight(int aprilTagId) {
    Pose3d tagPose =
        field
            .getTagPose(aprilTagId)
            .orElseThrow(() -> new IllegalArgumentException("Invalid AprilTag ID: " + aprilTagId));

    double tagX = tagPose.getX();
    double tagY = tagPose.getY();
    double tagYaw = tagPose.getRotation().getZ(); // Rotation in radians

    // Compute right reef position
    double reefX =
        tagX
            + REEF_X_OFFSET * Math.cos(tagYaw + Math.PI / 2) // Right shift
            + REEF_Y_OFFSET * Math.cos(tagYaw); // Forward shift
    double reefY =
        tagY
            + REEF_X_OFFSET * Math.sin(tagYaw + Math.PI / 2) // Right shift
            + REEF_Y_OFFSET * Math.sin(tagYaw); // Forward shift

    return new Pose2d(
        reefX,
        reefY,
        new Rotation2d(Math.toRadians(Math.ceil(Math.toDegrees(tagYaw - Math.PI)) + 0.01)));
  }

  public Pose2d getReefRightClose(int aprilTagId) {
    Pose3d tagPose =
        field
            .getTagPose(aprilTagId)
            .orElseThrow(() -> new IllegalArgumentException("Invalid AprilTag ID: " + aprilTagId));

    double tagX = tagPose.getX();
    double tagY = tagPose.getY();
    double tagYaw = tagPose.getRotation().getZ(); // Rotation in radians

    // Compute right reef position
    double reefX =
        tagX
            + REEF_X_OFFSET * Math.cos(tagYaw + Math.PI / 2) // Right shift
            + (REEF_Y_OFFSET - 0.4) * Math.cos(tagYaw); // Forward shift
    double reefY =
        tagY
            + REEF_X_OFFSET * Math.sin(tagYaw + Math.PI / 2) // Right shift
            + (REEF_Y_OFFSET - 0.4) * Math.sin(tagYaw); // Forward shift

    return new Pose2d(
        reefX,
        reefY,
        new Rotation2d(Math.toRadians(Math.ceil(Math.toDegrees(tagYaw - Math.PI)) + 0.01)));
  }

  public Pose2d getReefLeft(int aprilTagId) {
    Pose3d tagPose =
        field
            .getTagPose(aprilTagId)
            .orElseThrow(() -> new IllegalArgumentException("Invalid AprilTag ID: " + aprilTagId));

    double tagX = tagPose.getX();
    double tagY = tagPose.getY();
    double tagYaw = tagPose.getRotation().getZ(); // Rotation in radians

    // Compute left reef position
    double reefX =
        tagX
            + REEF_X_OFFSET * Math.cos(tagYaw - Math.PI / 2) // Left shift
            + REEF_Y_OFFSET * Math.cos(tagYaw); // Forward shift
    double reefY =
        tagY
            + REEF_X_OFFSET * Math.sin(tagYaw - Math.PI / 2) // Left shift
            + REEF_Y_OFFSET * Math.sin(tagYaw); // Forward shift

    return new Pose2d(
        reefX,
        reefY,
        new Rotation2d(Math.toRadians(Math.ceil(Math.toDegrees(tagYaw - Math.PI)) + 0.01)));
  }

  public Pose2d getReefLeftClose(int aprilTagId) {
    Pose3d tagPose =
        field
            .getTagPose(aprilTagId)
            .orElseThrow(() -> new IllegalArgumentException("Invalid AprilTag ID: " + aprilTagId));

    double tagX = tagPose.getX();
    double tagY = tagPose.getY();
    double tagYaw = tagPose.getRotation().getZ(); // Rotation in radians

    // Compute left reef position
    double reefX =
        tagX
            + REEF_X_OFFSET * Math.cos(tagYaw - Math.PI / 2) // Left shift
            + (REEF_Y_OFFSET - 0.4) * Math.cos(tagYaw); // Forward shift
    double reefY =
        tagY
            + REEF_X_OFFSET * Math.sin(tagYaw - Math.PI / 2) // Left shift
            + (REEF_Y_OFFSET - 0.4) * Math.sin(tagYaw); // Forward shift

    return new Pose2d(
        reefX,
        reefY,
        new Rotation2d(Math.toRadians(Math.ceil(Math.toDegrees(tagYaw - Math.PI)) + 0.01)));
  }

  public Pose2d getReefCenter(int aprilTagId) {
    Pose3d tagPose =
        field
            .getTagPose(aprilTagId)
            .orElseThrow(() -> new IllegalArgumentException("Invalid AprilTag ID: " + aprilTagId));

    double tagX = tagPose.getX();
    double tagY = tagPose.getY();
    double tagYaw = tagPose.getRotation().getZ(); // Rotation in radians

    // Compute left reef position
    double reefX =
        tagX
            + 0 * Math.cos(tagYaw - Math.PI / 2) // Left shift
            + REEF_Y_OFFSET * Math.cos(tagYaw); // Forward shift
    double reefY =
        tagY
            + 0 * Math.sin(tagYaw - Math.PI / 2) // Left shift
            + REEF_Y_OFFSET * Math.sin(tagYaw); // Forward shift

    return new Pose2d(
        reefX,
        reefY,
        new Rotation2d(Math.toRadians(Math.ceil(Math.toDegrees(tagYaw - Math.PI)) + 0.01)));
  }

  public Pose2d getReefCenterClose(int aprilTagId) {
    Pose3d tagPose =
        field
            .getTagPose(aprilTagId)
            .orElseThrow(() -> new IllegalArgumentException("Invalid AprilTag ID: " + aprilTagId));

    double tagX = tagPose.getX();
    double tagY = tagPose.getY();
    double tagYaw = tagPose.getRotation().getZ(); // Rotation in radians

    // Compute left reef position
    double reefX =
        tagX
            + 0 * Math.cos(tagYaw - Math.PI / 2) // Left shift
            + (REEF_Y_OFFSET - 0.3) * Math.cos(tagYaw); // Forward shift
    double reefY =
        tagY
            + 0 * Math.sin(tagYaw - Math.PI / 2) // Left shift
            + (REEF_Y_OFFSET - 0.3) * Math.sin(tagYaw); // Forward shift

    return new Pose2d(
        reefX,
        reefY,
        new Rotation2d(Math.toRadians(Math.ceil(Math.toDegrees(tagYaw - Math.PI)) + 0.01)));
  }

  public Pose2d getHumanPose2d(int aprilTagId) {
    Pose3d tagPose =
        field
            .getTagPose(aprilTagId)
            .orElseThrow(() -> new IllegalArgumentException("Invalid AprilTag ID: " + aprilTagId));

    double tagX = tagPose.getX();
    double tagY = tagPose.getY();
    double tagYaw = tagPose.getRotation().getZ(); // Rotation in radians

    // Compute left reef position
    double reefX =
        tagX
            + 0.3 * Math.cos(tagYaw - Math.PI / 2) // Left shift
            + (REEF_Y_OFFSET - 0.2) * Math.cos(tagYaw); // Forward shift
    double reefY =
        tagY
            + 0.3 * Math.sin(tagYaw - Math.PI / 2) // Left shift
            + (REEF_Y_OFFSET - 0.2) * Math.sin(tagYaw); // Forward shift

    return new Pose2d(
        reefX, reefY, new Rotation2d(Math.toRadians(Math.ceil(Math.toDegrees(tagYaw)) + 0.01)));
  }

  /** Returns the current odometry rotation. */
  @AutoLogOutput(key = "Robot/Drive/Rotation2d")
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
