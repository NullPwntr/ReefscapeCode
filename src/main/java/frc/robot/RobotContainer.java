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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.CoralCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
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
  private final Coral coral;
  private final Algae algae;
  private final Elevator elevator;
  private final LEDs led;

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
        led = new LEDs();

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
        led = new LEDs();
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
        led = new LEDs();

        break;
    }


    NamedCommands.registerCommand(
        "Move Backwards L4", DriveCommands.driveBackwards(drive).withTimeout(0.3));
    NamedCommands.registerCommand(
        "Elevator Set L4",
        ElevatorCommands.SetSetpoint(elevator, RobotConstants.ElevatorSubsystem.Setpoints.L3));
    NamedCommands.registerCommand(
        "Elevator Set Home",
        ElevatorCommands.SetSetpoint(
            elevator, RobotConstants.ElevatorSubsystem.Setpoints.MinimumHeight));
    NamedCommands.registerCommand(
        "Coral Set L4",
        Commands.runOnce(
            () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.TopScoring), coral));
    NamedCommands.registerCommand(
        "Coral Set L1",
        Commands.runOnce(
            () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.NormalScoring), coral));
    NamedCommands.registerCommand(
        "Coral Set Home",
        Commands.runOnce(
            () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.Home), coral));
    NamedCommands.registerCommand(
        "Coral Start Outtake", CoralCommands.OuttakeSlow(coral).withTimeout(1));
    NamedCommands.registerCommand("Coral Stop Motor", CoralCommands.stopMotor(coral));
    NamedCommands.registerCommand(
        "Coral Start Commands", CoralCommands.SetIsRunningCommand(coral, true));
    NamedCommands.registerCommand(
        "Coral Stop Commands", CoralCommands.SetIsRunningCommand(coral, false));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
            () -> driverController.getLeftTriggerAxis(),
            driverController.rightBumper(),
            driverController.leftBumper()));

    ////////////////////////////////////////////////////////// V-- DRIVER --V
    // ///////////////////////////////////////////////////////////////////////////

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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
                AlgaeCommands.setSecondarySetpoint(algae, 30)))
        .onFalse(
            Commands.sequence(
                AlgaeCommands.stopMotor(algae),
                AlgaeCommands.setSecondarySetpoint(algae, 0),
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
                AlgaeCommands.setSecondarySetpoint(algae, 30)))
        .onFalse(
            Commands.sequence(
                AlgaeCommands.stopMotor(algae),
                AlgaeCommands.setSecondarySetpoint(algae, 0),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.MinimumHeight)));

    // Top algae intake
    operatorController
        .y()
        .onTrue(
            Commands.sequence(
                AlgaeCommands.SetIsRunningCommand(algae, true),
                ElevatorCommands.SetSetpoint(elevator, 5),
                AlgaeCommands.Intake(algae),
                AlgaeCommands.setSecondarySetpoint(algae, 60)))
        .onFalse(
            Commands.sequence(
                AlgaeCommands.stopMotor(algae),
                ElevatorCommands.SetSetpoint(elevator, 0),
                AlgaeCommands.setSecondarySetpoint(algae, 0),
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

    operatorController
        .leftBumper()
        .onTrue(
            Commands.sequence(
                AlgaeCommands.SetIsRunningCommand(algae, true),
                AlgaeCommands.SetIsLBHeld(algae, true),
                AlgaeCommands.setSecondarySetpoint(algae, 40),
                AlgaeCommands.Outtake(algae)))
        .onFalse(
            Commands.sequence(
                AlgaeCommands.SetIsRunningCommand(algae, false),
                AlgaeCommands.SetIsLBHeld(algae, false),
                AlgaeCommands.setSecondarySetpoint(algae, 0),
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
                // CoralCommands.Outtake(coral).withTimeout(0.5),
                // CoralCommands.stopMotor(coral),
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
                // CoralCommands.Outtake(coral).withTimeout(0.5),
                // CoralCommands.stopMotor(coral),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.MinimumHeight),
                Commands.runOnce(() -> coral.setIsRunningCommand(false), coral) // Second action
                ));

    operatorController
        .pov(90)
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> coral.setIsRunningCommand(true), coral),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.L1),
                Commands.runOnce(
                    () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.Home + 4),
                    coral) // First action
                ))
        .onFalse(
            Commands.sequence(
                CoralCommands.OuttakeSlow(coral).withTimeout(3),
                Commands.runOnce(
                        () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.Home),
                        coral)
                    .withTimeout(2), // First action
                CoralCommands.stopMotor(coral),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.MinimumHeight),
                Commands.runOnce(() -> coral.setIsRunningCommand(false), coral) // Second action
                ));

    // dont ask
    operatorController.pov(0).onTrue(DriveCommands.driveBackwards(drive).withTimeout(0.3));
    operatorController
        .pov(0)
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.25),
                Commands.runOnce(() -> coral.setIsRunningCommand(true), coral),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.L3),
                // new WaitCommand(1),
                Commands.runOnce(
                    () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.TopScoring),
                    coral) // First action
                ))
        .onFalse(
            Commands.sequence(
                // Commands.waitSeconds(0.3), // Wait for 2 seconds
                // CoralCommands.OuttakeSlow(coral).withTimeout(1),
                Commands.runOnce(
                        () -> coral.setSetpoint(RobotConstants.CoralSubsystem.Setpoints.Home),
                        coral)
                    .withTimeout(0), // First action
                // CoralCommands.stopMotor(coral),
                ElevatorCommands.SetSetpoint(
                    elevator, RobotConstants.ElevatorSubsystem.Setpoints.MinimumHeight),
                Commands.runOnce(() -> coral.setIsRunningCommand(false), coral) // Second action
                ));

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

    // debugController.y().onTrue(DriveCommands.driveToReefRight());

    // .onFalse(
    //     Commands.runOnce(
    //         () -> {
    //             ElevatorCommands.SetSetpoint(elevator,
    // RobotConstants.ElevatorSubsystem.Setpoints.Home));
    //         },
    //         coral));

    // Starts Intaking for algae when A button is pressed (A)
    // operatorController
    //     .a()
    //     .onTrue(AlgaeCommands.Intake(algae))
    //     .onFalse(AlgaeCommands.stopMotor(algae));

    // // Throws algae when X button is pressed (X)
    // operatorController
    //     .x()
    //     .onTrue(AlgaeCommands.Outtake(algae))
    //     .onFalse(AlgaeCommands.stopMotor(algae));

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // OPERATOR CONTROLS PLAN:-
    // A - Coral Intake (Toggle)
    // Y - Algae Intake (Toggle)
    // POV UP - L4 Scoring (Angle on hold, Score on release if has coral)
    // POV RIGHT - L3 Scoring (Angle on hold, Score on release if has coral)
    // POV DOWN - L2 Scoring (Angle on hold, Score on release if has coral)
    // POV LEFT - L1 (Trough) Scoring
    // RB + LB - Net Scoring Angle (Elevator + Arms) for Algae
    // X - Throw Algae (For Processor + Net + Incase Stuck) (Hold)
    // B - Throw Coral Incase Stuck (Hold)
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
