// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {
  // Motors
  public TalonFX coralIntake = new TalonFX(RobotConstants.CoralSubsystem.IntakeMotorId);
  public TalonFX coralAngleMotor = new TalonFX(RobotConstants.CoralSubsystem.AngleSystem.MotorId);

  // Sensors
  public CANrange sensor = new CANrange(RobotConstants.CoralSubsystem.CANRangeId);
  public CANcoder cancoder =
      new CANcoder(RobotConstants.CoralSubsystem.AngleSystem.AngleCANCoderId);

  // Motion Controls
  private final PIDController pid =
      new PIDController(
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kP,
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kI,
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kD);

  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kS,
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kG,
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kV,
          RobotConstants.CoralSubsystem.AngleSystem.PIDFF.kA); // kS, kG, kV, kA

  // Variables
  @AutoLogOutput(key = "Coral/AngePIDOutput")
  double output = 0.0;

  @AutoLogOutput(key = "Coral/AngePIDSetpoint")
  double setpoint = 0.0;

  boolean isRunningCommand = false;
  boolean isRBHeld = false;

  public Coral() {
    // Motor Config //
    coralIntake.getConfigurator().apply(RobotConstants.CoralSubsystem.Config.IntakeConfig);
    coralAngleMotor.getConfigurator().apply(RobotConstants.CoralSubsystem.Config.AngleConfig);

    // Current Limiter //
    coralAngleMotor
        .getConfigurator()
        .apply(RobotConstants.CoralSubsystem.Config.AngleCurrentConfig);
    //  //  //  //  //  //

    coralAngleMotor.getConfigurator().setPosition(0.0);

    pid.setSetpoint(0);
    pid.setTolerance(0.2);
  }

  /** Returns the current sensor distance */
  @AutoLogOutput(key = "Coral/Sensor/Distance")
  public double getSensorDistance() {
    return sensor.getDistance().getValueAsDouble();
  }

  /** Returns true if the coral subsystem senses a coral, reutrns false otherwise */
  @AutoLogOutput(key = "Coral/HasCoral")
  public boolean hasCoral() {
    return (sensor.getDistance().getValueAsDouble()
            <= RobotConstants.CoralSubsystem.hasCoralThreshold
        && sensor.getAmbientSignal().getValueAsDouble() <= 10);
  }

  /** Returns the current coral arm angle position (CANCoder [0-100]) */
  @AutoLogOutput(key = "Coral/Angle/Position")
  public double getCANCoderPosition() {
    return (cancoder.getPosition().getValueAsDouble() + 0.000244140625) * 175; // offset
  }
  /** Returns the current coral arm angle position (CANCoder [0-100]) */
  @AutoLogOutput(key = "Coral/Angle/PositionRAW")
  public double getCANCoderPositionRAW() {
    return cancoder.getPosition().getValueAsDouble(); // offset
  }

  /** Changes the coral angle setpoint */
  public void setSetpoint(double Setpoint) {
    setpoint = Setpoint;
  }

  /**
   * Method that indicates to the subsystem that there is a command currently using the current
   * subsystem
   */
  public void setIsRunningCommand(boolean flag) {
    isRunningCommand = flag;
  }

  public void setIsRBHeld(boolean flag) {
    isRBHeld = flag;
  }

  /** Returns the current coral angle position (motor encoder x 4) */
  @AutoLogOutput(key = "Coral/CurrentAnglePosition")
  public double getCoralPosition() {
    return coralAngleMotor.getPosition().getValueAsDouble() * 4.0;
  }

  /** Updates the LED state based on the sensors */
  public void updateLEDs() {
    if (hasCoral()) {
      LEDs.currentColor = "WHITE";
      if (isRunningCommand == false) {
        setpoint = RobotConstants.CoralSubsystem.Setpoints.HumanIntake;
      }
    } else {
      LEDs.currentColor = "DEFAULT";
      if (isRunningCommand == false) {
        setpoint = RobotConstants.CoralSubsystem.Setpoints.HumanIntake;
      }
    }
  }

  @Override
  public void periodic() {
    output = pid.calculate(getCANCoderPosition());

    pid.setSetpoint(setpoint);

    coralAngleMotor.setVoltage(
        MathUtil.clamp(
                    output,
                    RobotConstants.CoralSubsystem.AngleSystem.ReturnMaxSpeed,
                    RobotConstants.CoralSubsystem.AngleSystem.LaunchMaxSpeed)
                * 12.0
            + feedforward.calculate(setpoint, output));

    updateLEDs();

    Logger.recordOutput(
        "Coral/CurrentAnglePosition", coralAngleMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Coral/CORALANGLESETPOINT", pid.getSetpoint());
  }
}
