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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  // public static Command joystickDrive(
  //     Drive drive,
  //     DoubleSupplier xSupplier,
  //     DoubleSupplier ySupplier,
  //     DoubleSupplier omegaSupplier,
  //     DoubleSupplier increaseSpeedSupplier,
  //     DoubleSupplier decreaseSpeedSupplier) {
  //   return Commands.run(
  //       () -> {
  //         // Get linear velocity
  //         Translation2d linearVelocity =
  //             getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

  //         // Apply rotation deadband

  //         double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

  //         // Square rotation value for more precise control
  //         omega = Math.copySign(omega * omega, omega);

  //         double X = linearVelocity.getX();
  //         double Y = linearVelocity.getY();
  //         double OMEGA = omega;

  //         if (increaseSpeedSupplier.getAsDouble() > 0.05) {
  //           X *= (1 + MathUtil.clamp((increaseSpeedSupplier.getAsDouble()), 0, 1)) * 2.5;
  //           Y *= (1 + MathUtil.clamp((increaseSpeedSupplier.getAsDouble()), 0, 1)) * 2.5;
  //           OMEGA *= (1 + MathUtil.clamp((increaseSpeedSupplier.getAsDouble()), 0, 1)) * 2.5;
  //         } else if (decreaseSpeedSupplier.getAsDouble() > 0.05) {
  //           X /= (1 + MathUtil.clamp((decreaseSpeedSupplier.getAsDouble()), 0, 1)) * 2.5;
  //           Y /= (1 + MathUtil.clamp((decreaseSpeedSupplier.getAsDouble()), 0, 1)) * 2.5;
  //           OMEGA /= (1 + MathUtil.clamp((decreaseSpeedSupplier.getAsDouble()), 0, 1)) * 2.5;
  //         }

  //         // Convert to field relative speeds & send command
  //         ChassisSpeeds speeds =
  //             new ChassisSpeeds(
  //                 X * drive.getMaxLinearSpeedMetersPerSec(),
  //                 Y * drive.getMaxLinearSpeedMetersPerSec(),
  //                 OMEGA * drive.getMaxAngularSpeedRadPerSec());
  //         boolean isFlipped =
  //             DriverStation.getAlliance().isPresent()
  //                 && DriverStation.getAlliance().get() == Alliance.Red;
  //         drive.runVelocity(
  //             ChassisSpeeds.fromFieldRelativeSpeeds(
  //                 speeds,
  //                 isFlipped
  //                     ? drive.getRotation().plus(new Rotation2d(Math.PI))
  //                     : drive.getRotation()));
  //       },
  //       drive);
  // }
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier rightTriggerSupplier,
      DoubleSupplier leftTriggerSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity from the joysticks
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply deadband to rotation and square for finer control
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
          omega = Math.copySign(omega * omega, omega);

          // Calculate speed multiplier based on trigger inputs:
          // - Default multiplier is 0.7 (70% of maximum speed)
          // - Left trigger linearly reduces the multiplier (down to 0.3)
          // - Right trigger linearly increases the multiplier (up to 1.0)
          double baseMultiplier = RobotConstants.SwerveSettings.baseSpeedPercentage;
          double minimumSpeed = RobotConstants.SwerveSettings.minimumSpeedPercentage;
          double maximumSpeed = RobotConstants.SwerveSettings.maximumSpeedPercentage;

          double leftReduction =
              leftTriggerSupplier.getAsDouble()
                  * (baseMultiplier
                      - minimumSpeed); // 0.7 -> 0.3 when fully pressed (baseMultiplier -
          // minimumSpeed = 0.4)
          double rightIncrease =
              rightTriggerSupplier.getAsDouble()
                  * (maximumSpeed
                      - baseMultiplier); // 0.7 -> 1.0 when fully pressed (maximumSpeed -
          // baseMultiplier = 0.3)

          double speedMultiplier =
              baseMultiplier
                  - leftReduction
                  + rightIncrease; // Adding up the incrase/reduction values to the base multiplier
          // thus giving us a proper multiplier for both triggers
          speedMultiplier =
              MathUtil.clamp(
                  speedMultiplier,
                  minimumSpeed,
                  maximumSpeed); // Make [SpeedMultiplier = minimumSpeed] when both triggers are
          // held down (0.7 - 0.4 + 0.3 => 0 ---clamp---> 0.3)

          // Disable speed modifications in autonomous
          speedMultiplier = DriverStation.isAutonomousEnabled() ? 1 : speedMultiplier;

          // Calculate the maximum linear/angular speed based on our multiplier
          double maxLinearSpeed = drive.getMaxLinearSpeedMetersPerSec() * speedMultiplier;
          double maxAngularSpeed = drive.getMaxAngularSpeedRadPerSec() * speedMultiplier;

          // Convert to field-relative speeds & send the command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * maxLinearSpeed,
                  linearVelocity.getY() * maxLinearSpeed,
                  omega * maxAngularSpeed);

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }


  //   public Command driveToReefRight() {
  //   PathConstraints constraints = new PathConstraints(
  //       1, 1,
  //       Units.degreesToRadians(180), Units.degreesToRadians(180));
  //   Pose2d targetPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(10));
  //   return AutoBuilder.pathfindToPose(targetPose, constraints);
  // }

  // public Command driveToReefLeft() {
  //   PathConstraints constraints = new PathConstraints(
  //       1, 1,
  //       Units.degreesToRadians(180), Units.degreesToRadians(180));
  //   Pose2d targetPose = new Pose2d();
  //   return AutoBuilder.pathfindToPose(targetPose, constraints);
  // }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
