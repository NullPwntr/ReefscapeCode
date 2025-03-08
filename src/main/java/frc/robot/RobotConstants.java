package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class RobotConstants {
  public class Controllers {
    public static final int DriverPortId = 0;
    public static final int OperatorPortId = 1;

    public static final int DebugPortId = 5; // Future use
  }

  public class SwerveSettings {
    public static final double baseSpeedPercentage =
        0.5; // Base speed without modifications; (X * 100)% of the MAXIMUM speed (5.41m/s)

    public static final double maximumSpeedPercentage =
        1.0; // Keep at 1.0 for absolute maximum speed (5.41m/s)
    public static final double minimumSpeedPercentage =
        0.05; // (X * 100)% of the MAXIMUM speed (5.41m/s)
  }

  public class CoralSubsystem {
    public static final int IntakeMotorId = 30;

    public static final int CANRangeId = 51; // Distance sensor
    public static final double hasCoralThreshold = 0.1; // The value where the sensor sees a coral

    public static final double IntakeSpeed = 0.3; // Default Intake Speed
    public static final double OuttakeSpeed = -0.3; // Default Outtake Speed (Negative value)

    public class AngleSystem {
      public static final int MotorId = 34;
      public static final int AngleCANCoderId = 59;

      public static final double LaunchMaxSpeed = 0.4;
      public static final double ReturnMaxSpeed = -0.3;

      public class PIDFF {
        public static final double kP = 0.01;
        public static final double kI = 0;
        public static final double kD = 0.00001;

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;
      }
    }

    public class Setpoints { // Change to CANCoder angles (These are motor positions)
      public static final double NormalScoring = 101.5;
      public static final double MiddleScoring = 101.5;
      public static final double TopScoring = 115;

      public static final double HumanIntake = 24.5; // 21.0 home
      public static final double Home = 0.0;

      public static final double Center = 26.2 * 4.0 / 2.0; // (Top of the elevator)
    }

    public class Config {
      public static final MotorOutputConfigs IntakeConfig =
          new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Brake)
              .withInverted(InvertedValue.Clockwise_Positive);

      public static final MotorOutputConfigs AngleConfig =
          new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Brake)
              .withInverted(InvertedValue.CounterClockwise_Positive);

      public static final CurrentLimitsConfigs AngleCurrentConfig =
          new CurrentLimitsConfigs()
              .withStatorCurrentLimit(40.0)
              .withStatorCurrentLimitEnable(true);
    }
  }

  public class AlgaeSubsystem {
    public static final int IntakeMotorId = 31;
    public static final int SecondaryArmMotorId = 41; // The one connected to the
    // pivot of the primary arm

    public static final int SecondaryArmCANCoderId = 28;
    public static final SensorDirectionValue SecondaryArmCANCoderDirection =
        SensorDirectionValue.Clockwise_Positive;

    public static final int CANRangeId = 50; // Distance sensor
    public static final double hasAlgaeThreshold = 0.1; // The value where the sensor sees an algae

    public static final double IntakeSpeed = 0.75; // Default Intake Speed
    public static final double IntakeHoldbackSpeed =
        0.08; // Intake speed when holding an algae (This is to hold the algae in place).

    public static final double OuttakeSpeed = -0.5; // Default Outtake Speed (Negative value)

    public class SecondaryArm {
      public class PIDFF {
        public static final double kP = 0.0035;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;
      }

      public class Angles {
        public static final double Home = 0.0;

        public static final double GroundIntake = 87.15;

        public static final double ReefTopIntake = 55.5;
        public static final double ReefBottomIntake = 55.5;

        public static final double ThrowAngle = 60;
      }
    }

    public class Config {
      public static final MotorOutputConfigs IntakeConfig =
          new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Brake)
              .withInverted(InvertedValue.Clockwise_Positive);

      public static final MotorOutputConfigs SecondaryConfig =
          new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Brake)
              .withInverted(InvertedValue.CounterClockwise_Positive);

      public static final CANcoderConfiguration CANCoderConfig =
          new CANcoderConfiguration()
              .withMagnetSensor(
                  new MagnetSensorConfigs()
                      .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
    }
  }

  public class ElevatorSubsystem {
    public static final int TopMotorId = 32;
    public static final int BottomMotorId = 33;

    public static final int ElevatorCANCoderId = 29;

    public static final double AscendMaxSpeed = 0.5; // 0.5
    public static final double DescendMaxSpeed = -0.4; // -0.4 (Negative value)

    public class PIDFF {
      public static final double kP = 0.05; // 0.1;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double kS = 0;
      public static final double kG = 0.34; // Manually calculated
      public static final double kV = 0;
      public static final double kA = 0;
    }

    public class Setpoints {
      public static final double MaxHeight = 77; // based on the motor sensor
      public static final double MinimumHeight = 0; // based on the motor sensor

      public static final double L0 = 0; // -1.0 is basically 0
      public static final double L1 = 0; // -1.0 is basically 0
      public static final double L2 = 21.0; // 22.5
      public static final double L3 = 60.2; // 60.5
      public static final double TopAlgae = 45.0; // not tested yet
      public static final double BottomAlgae = 26.0; // not tested yet

      public static final double AlgaeScoringHeight = 0.0; // not tested yet (probably MaxHeight)
      public static final double ProcessorScoringHeight =
          0.0; // not tested yet (probably not needed since we have arms for algae)
    }

    public class Config {
      public static final MotorOutputConfigs MotorsConfig =
          new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Brake)
              .withInverted(InvertedValue.CounterClockwise_Positive);

      public static final CurrentLimitsConfigs CurrentConfig =
          new CurrentLimitsConfigs()
              .withStatorCurrentLimit(40.0)
              .withStatorCurrentLimitEnable(true);

      public static final CANcoderConfiguration CANCoderConfig =
          new CANcoderConfiguration()
              .withMagnetSensor(
                  new MagnetSensorConfigs()
                      .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
    }
  }

  public class LED {
    public static final int CANdleId = 54;
    public static final int ledCount = 149;
  }
}
