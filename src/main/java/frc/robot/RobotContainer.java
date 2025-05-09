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

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.CoralCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.VisionCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;

  private final Coral coral;
  private final Algae algae;
  private final Elevator elevator;
  //   private final Climb climb;
  //   private final LEDs led;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(RobotConstants.Controllers.DriverPortId);
  private final CommandXboxController operatorController =
      new CommandXboxController(RobotConstants.Controllers.OperatorPortId);
  private final CommandXboxController debugController =
      new CommandXboxController(RobotConstants.Controllers.DebugPortId);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        coral = new Coral();
        algae = new Algae();
        elevator = new Elevator();
        // climb = new Climb();
        // led = new LEDs();

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        coral = new Coral();
        algae = new Algae();
        elevator = new Elevator();
        // climb = new Climb();
        // led = new LEDs();

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        coral = new Coral();
        algae = new Algae();
        elevator = new Elevator();
        // climb = new Climb();
        // led = new LEDs();

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }

    // NamedCommands for autonomous //

    NamedCommands.registerCommand(
        "Move Backwards L4", DriveCommands.driveBackwards(drive).withTimeout(0.3));
    NamedCommands.registerCommand(
        "Elevator Set L4",
        ElevatorCommands.SetSetpoint(elevator, RobotConstants.ElevatorSubsystem.Setpoints.L3));
    NamedCommands.registerCommand(
        "Elevator Set L3",
        ElevatorCommands.SetSetpoint(elevator, RobotConstants.ElevatorSubsystem.Setpoints.L2));
    NamedCommands.registerCommand(
        "Elevator Set Home",
        ElevatorCommands.SetSetpoint(
            elevator, RobotConstants.ElevatorSubsystem.Setpoints.MinimumHeight));
    NamedCommands.registerCommand(
        "Coral Set L4",
        Commands.runOnce(
            () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.TopScoring), coral));
    NamedCommands.registerCommand(
        "Coral Set L3",
        Commands.runOnce(
            () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.MiddleScoring), coral));
    NamedCommands.registerCommand(
        "Coral Set L1",
        Commands.runOnce(
            () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.NormalScoring), coral));
    NamedCommands.registerCommand(
        "Coral Set Home",
        Commands.runOnce(
            () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.Home), coral));
    NamedCommands.registerCommand(
        "Coral Set Intake Angle",
        Commands.runOnce(
            () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.HumanIntake), coral));
    NamedCommands.registerCommand(
        "Coral Start Outtake", CoralCommands.OuttakeSlow(coral).withTimeout(0.5));
    NamedCommands.registerCommand("Coral Start Intake", CoralCommands.Intake(coral));
    NamedCommands.registerCommand("Coral Stop Motor", CoralCommands.stopMotor(coral));

    NamedCommands.registerCommand(
        "Coral Start Commands", CoralCommands.SetIsRunningCommand(coral, true));
    NamedCommands.registerCommand(
        "Coral Stop Commands", CoralCommands.SetIsRunningCommand(coral, false));
    NamedCommands.registerCommand(
        "Algae Start Commands", AlgaeCommands.SetIsRunningCommand(algae, true));
    NamedCommands.registerCommand(
        "Algae Stop Commands", AlgaeCommands.SetIsRunningCommand(algae, false));
    NamedCommands.registerCommand("Algae Start LB", AlgaeCommands.SetIsLBHeld(algae, true));
    NamedCommands.registerCommand("Algae Stop LB", AlgaeCommands.SetIsLBHeld(algae, false));

    NamedCommands.registerCommand(
        "Align To Closest Right Reef", DriveCommands.driveToReefRight(drive));
    NamedCommands.registerCommand(
        "Align To Closest Right Reef Close", DriveCommands.driveToReefRightClose(drive));
    NamedCommands.registerCommand(
        "Align To Closest Left Reef", DriveCommands.driveToReefLeft(drive));
    NamedCommands.registerCommand(
        "Align To Closest Left Reef Close", DriveCommands.driveToReefLeftClose(drive));
    NamedCommands.registerCommand(
        "Align To Closest Center Reef", DriveCommands.driveToReefCenter(drive, elevator, algae));
    NamedCommands.registerCommand(
        "Align To Closest Center Reef Close", DriveCommands.driveToReefCenterClose(drive));
    NamedCommands.registerCommand(
        "Align To Closest Human Intake", DriveCommands.driveToHumanIntake(drive));
    NamedCommands.registerCommand(
        "Intake Until Has Coral", CoralCommands.intakeUntilHasCoral(coral));
    NamedCommands.registerCommand("Turn Off Vision", VisionCommands.TurnOffVisionPoseEstimation());
    NamedCommands.registerCommand("Turn On Vision", VisionCommands.TurnOnVisionPoseEstimation());
    NamedCommands.registerCommand("Turn On Reef Vision", VisionCommands.TurnOnReefPoseEstimation());
    NamedCommands.registerCommand(
        "Turn On Human Vision", VisionCommands.TurnOnHumanPoseEstimation());
    NamedCommands.registerCommand("Algae Stop Motor", AlgaeCommands.stopMotor(algae));
    NamedCommands.registerCommand(
        "Algae Set Home",
        AlgaeCommands.setSecondarySetpoint(
            algae, RobotConstants.AlgaeSubsystem.SecondaryArm.Angles.Home));
    NamedCommands.registerCommand(
        "Algae Outtake",
        Commands.sequence(
            AlgaeCommands.SetIsRunningCommand(algae, true),
            AlgaeCommands.SetIsLBHeld(algae, true),
            AlgaeCommands.setSecondarySetpoint(
                algae, RobotConstants.AlgaeSubsystem.SecondaryArm.Angles.ThrowAngle),
            new WaitCommand(0.2),
            AlgaeCommands.Outtake(algae),
            new WaitCommand(1),
            AlgaeCommands.stopMotor(algae),
            new WaitCommand(2),
            AlgaeCommands.SetIsRunningCommand(algae, false),
            AlgaeCommands.SetIsLBHeld(algae, false),
            AlgaeCommands.setSecondarySetpoint(
                algae, RobotConstants.AlgaeSubsystem.SecondaryArm.Angles.Home)));

    // // // // // // //

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData(
        "Zero Gyro",
        new InstantCommand(
                () -> {
                  drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
                })
            .ignoringDisable(true));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverController.getRightTriggerAxis(),
            () -> driverController.getLeftTriggerAxis()));

    // slow intake for stable coral holding (only runs when no other coral commands are running)
    coral.setDefaultCommand(CoralCommands.SlowIntakeForCoral(coral));

    ////////////////////////////////////////////////////////// V-- DRIVER --V
    // ///////////////////////////////////////////////////////////////////////////

    // Lock to 0° when A button is held
    // driverController
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driverController.x().onTrue(Commands.sequence(DriveCommands.driveToHumanIntake(drive)));

    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driverController
        .leftBumper()
        .onTrue(
            Commands.sequence(
                DriveCommands.driveToReefLeft(drive), DriveCommands.driveToReefLeftClose(drive)));
    driverController
        .rightBumper()
        .onTrue(
            Commands.sequence(
                DriveCommands.driveToReefRight(drive), DriveCommands.driveToReefRightClose(drive)));
    driverController
        .a()
        .onTrue(
            Commands.sequence(
                DriveCommands.driveToReefCenter(drive, elevator, algae),
                DriveCommands.driveToReefCenterClose(drive)));
    ////////////////////////////////////////////////////////// V-- OPERATOR --V
    // ///////////////////////////////////////////////////////////////////////////

    // Bottom algae intake
    operatorController
        .a()
        .onTrue(
            Commands.sequence(
                AlgaeCommands.SetIsRunningCommand(algae, true),
                AlgaeCommands.Intake(algae),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.BottomAlgae),
                AlgaeCommands.setSecondarySetpoint(
                    algae, RobotConstants.AlgaeSubsystem.SecondaryArm.Angles.ReefBottomIntake)))
        .onFalse(
            Commands.sequence(
                AlgaeCommands.stopMotor(algae),
                AlgaeCommands.setSecondarySetpoint(
                    algae, RobotConstants.AlgaeSubsystem.SecondaryArm.Angles.Home),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.MinimumHeight),
                AlgaeCommands.SetIsRunningCommand(algae, false)));

    // Top algae intake
    operatorController
        .x()
        .onTrue(
            Commands.sequence(
                AlgaeCommands.Intake(algae),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.TopAlgae),
                AlgaeCommands.setSecondarySetpoint(
                    algae, RobotConstants.AlgaeSubsystem.SecondaryArm.Angles.ReefTopIntake)))
        .onFalse(
            Commands.sequence(
                AlgaeCommands.stopMotor(algae),
                AlgaeCommands.setSecondarySetpoint(
                    algae, RobotConstants.AlgaeSubsystem.SecondaryArm.Angles.Home),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.MinimumHeight)));

    // Ground algae intake
    operatorController
        .y()
        .onTrue(
            Commands.sequence(
                AlgaeCommands.SetIsRunningCommand(algae, true),
                AlgaeCommands.Intake(algae),
                AlgaeCommands.setSecondarySetpoint(
                    algae, RobotConstants.AlgaeSubsystem.SecondaryArm.Angles.GroundIntake)))
        .onFalse(
            Commands.sequence(
                AlgaeCommands.stopMotor(algae),
                AlgaeCommands.setSecondarySetpoint(
                    algae, RobotConstants.AlgaeSubsystem.SecondaryArm.Angles.Home),
                AlgaeCommands.SetIsRunningCommand(algae, false)));

    // Starts Intaking for coral when B button is pressed (B)
    operatorController
        .b()
        .onTrue(CoralCommands.Intake(coral))
        .onFalse(CoralCommands.stopMotor(coral));

    // Throws coral when RB button is pressed (RB)
    operatorController
        .rightBumper()
        .onTrue(CoralCommands.Outtake(coral))
        .onFalse(CoralCommands.stopMotor(coral));

    // Throws algae when LB button is pressed (LB)
    operatorController
        .leftBumper()
        .onTrue(
            Commands.sequence(
                AlgaeCommands.SetIsRunningCommand(algae, true),
                AlgaeCommands.SetIsLBHeld(algae, true),
                AlgaeCommands.setSecondarySetpoint(
                    algae, RobotConstants.AlgaeSubsystem.SecondaryArm.Angles.ThrowAngle),
                new WaitCommand(0.2),
                AlgaeCommands.Outtake(algae)))
        .onFalse(
            Commands.sequence(
                AlgaeCommands.SetIsRunningCommand(algae, false),
                AlgaeCommands.SetIsLBHeld(algae, false),
                AlgaeCommands.setSecondarySetpoint(
                    algae, RobotConstants.AlgaeSubsystem.SecondaryArm.Angles.Home),
                AlgaeCommands.stopMotor(algae)));

    operatorController
        .pov(270)
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> coral.setIsRunningCommand(true), coral),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.L2),
                Commands.runOnce(
                    () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.MiddleScoring),
                    coral) // First action
                ))
        .onFalse(
            Commands.sequence(
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.MinimumHeight),
                Commands.runOnce(() -> coral.setIsRunningCommand(false), coral) // Second action
                ));

    operatorController
        .pov(180)
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> coral.setIsRunningCommand(true), coral),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.L1),
                Commands.runOnce(
                    () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.NormalScoring),
                    coral) // First action
                ))
        .onFalse(
            Commands.sequence(
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.MinimumHeight),
                Commands.runOnce(() -> coral.setIsRunningCommand(false), coral) // Second action
                ));

    operatorController
        .pov(0)
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> coral.setIsRunningCommand(true), coral),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.L3),
                new WaitCommand(0.5),
                Commands.runOnce(
                    () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.TopScoring),
                    coral) // First action
                ))
        .onFalse(
            Commands.sequence(
                Commands.runOnce(
                        () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.Home),
                        coral)
                    .withTimeout(0), // First action
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.MinimumHeight),
                Commands.runOnce(() -> coral.setIsRunningCommand(false), coral) // Second action
                ));

    // debugController
    //     .b()
    //     .onTrue(VisionCommands.TurnOffVisionPoseEstimation())
    //     .onFalse(VisionCommands.TurnOnVisionPoseEstimation());

    // debugController
    //     .a()
    //     .onTrue(ElevatorCommands.SetSetpoint(elevator, 77))
    //     .onFalse(ElevatorCommands.SetSetpoint(elevator, 0));
    // debugController
    //     .b()
    //     .onTrue(
    //         Commands.sequence(
    //             AlgaeCommands.SetIsRunningCommand(algae, true),
    //             AlgaeCommands.SetIsNetScoring(algae, true),
    //             AlgaeCommands.setSecondarySetpoint(algae, 50),
    //             new WaitCommand(0.25),
    //             AlgaeCommands.setPrimarySetpoint(algae, 115),
    //             AlgaeCommands.setSecondarySetpoint(algae, 50),
    //             new WaitCommand(0.25),
    //             ElevatorCommands.SetSetpoint(elevator, 77)))
    //     .onFalse(
    //         Commands.sequence(
    //             ElevatorCommands.SetSetpoint(elevator, 0),
    //             new WaitCommand(2),
    //             AlgaeCommands.setSecondarySetpoint(algae, 125),
    //             new WaitCommand(1),
    //             AlgaeCommands.setPrimarySetpoint(algae, 0),
    //             AlgaeCommands.setSecondarySetpoint(algae, 0),
    //             AlgaeCommands.SetIsRunningCommand(algae, false),
    //             AlgaeCommands.SetIsNetScoring(algae, false)));

    // debugController.y().onTrue(DriveCommands.driveToReefLeft(drive));

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // OPERATOR CONTROLS PLAN:-
    // A - Bottom algae intake (Hold)
    // X - Top algae intake (Hold)
    // B - Coral human intake (Hold)
    // Y - Algae ground Intake (Hold)
    // POV UP - L4 Scoring (Hold)
    // POV LEFT - L3 Scoring (Hold)
    // POV DOWN - L2 Scoring (Hold)
    // POV RIGHT - L1 (Trough) Scoring (Hold) [UNUSED]
    // RB - Coral throw (Hold)
    // LB - Algae throw (Hold)
    // RT - Climber goes down (Hold)
    // LT - Climber goes up (Hold)
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
