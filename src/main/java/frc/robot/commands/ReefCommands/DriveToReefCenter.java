package frc.robot.commands.ReefCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drive;

public class DriveToReefCenter extends Command {
  private final Drive drive;
  private final Elevator elevator;
  private final Algae algae;
  private Pose2d targetPose;
  private int closesetTag;

  private double wantedElevatorSetpoint;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  private final XboxController driverController =
      new XboxController(RobotConstants.Controllers.DriverPortId);

  // PID constants (tune these)
  private static final double X_KP = 3;
  private static final double Y_KP = 3;
  private static final double THETA_KP = 3;

  public DriveToReefCenter(Drive drive, Elevator elevator, Algae algae) {
    this.drive = drive;
    this.elevator = elevator;
    this.algae = algae;

    xController = new PIDController(X_KP, 0, 0);
    yController = new PIDController(Y_KP, 0, 0);
    thetaController = new PIDController(THETA_KP, 0, 0);

    thetaController.enableContinuousInput(-Math.PI, Math.PI); // Handle wraparound

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();
    System.out.println("🚀 DriveToPosePID Started: Moving to " + targetPose);

    targetPose = drive.getClosesPose2dCenter();
    closesetTag = drive.getClosestReefAprilTagToRobot();
  }

  @Override
  public void execute() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      if (closesetTag % 2 == 1) {
        wantedElevatorSetpoint = RobotConstants.ElevatorSubsystem.Setpoints.TopAlgae;
      } else {
        wantedElevatorSetpoint = RobotConstants.ElevatorSubsystem.Setpoints.BottomAlgae;
      }
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      if (closesetTag % 2 == 1) {
        wantedElevatorSetpoint = RobotConstants.ElevatorSubsystem.Setpoints.BottomAlgae;
      } else {
        wantedElevatorSetpoint = RobotConstants.ElevatorSubsystem.Setpoints.TopAlgae;
      }
    }

    algae.setIsRunningCommand(true);
    algae.algaeIntake.set(RobotConstants.AlgaeSubsystem.IntakeSpeed);
    elevator.setSetpoint(wantedElevatorSetpoint);
    algae.setSecondaryArmSetpoint(
        RobotConstants.AlgaeSubsystem.SecondaryArm.Angles.ReefBottomIntake);

    Pose2d currentPose = drive.getPose(); // Get live field-relative pose

    // Calculate velocity outputs in FIELD-RELATIVE coordinates
    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double thetaSpeed =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Convert FIELD-RELATIVE speeds to ROBOT-RELATIVE speeds
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            // new ChassisSpeeds(-xSpeed, -ySpeed, 0),
            new ChassisSpeeds(-xSpeed, -ySpeed, thetaSpeed),
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());

    // Apply robot-relative movement
    drive.runVelocity(robotRelativeSpeeds);
  }

  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint())
        || driverController.getYButton();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("❌ DriveToPosePID Interrupted!");
    } else {
      System.out.println("✅ DriveToPosePID Completed Successfully!");
    }
    drive.runVelocity(new ChassisSpeeds(0, 0, 0)); // Stop robot when done
  }
}
