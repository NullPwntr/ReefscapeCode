package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RobotConstants {
  public class Controllers {
    public static final int DriverPortId = 0;
    public static final int OperatorPortId = 1;

    // public static final int DebugPortId = 5; // Future use
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
    public static final int GyroId = 0;

    public static final int CANRangeId = 0; // Distance sensor
    public static final double hasCoralThreshold = 0.0; // The value where the sensor sees a coral

    public static final double IntakeSpeed = 0.3; // Default Intake Speed
    public static final double OuttakeSpeed = -0.3; // Default Outtake Speed (Negative value)

    public class AngleSystem {
      public static final int MotorId = 34;

      public static final double LaunchMaxSpeed = 0.4;
      public static final double ReturnMaxSpeed = -0.1;

      public class PIDFF {
        public static final double kP = 0.08;
        public static final double kI = 0;
        public static final double kD = 0.003;

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;
      }
    }

    public class Setpoints { // Change to gyro angles (These are motor positions)
      public static final double NormalScoring = 24.0;
      public static final double TopScoring = 0.0; // Not tested yet

      public static final double HumanIntake = 0.0;

      public static final double Center = 26.2 / 2.0; // (Top of the elevator)
    }

    public class Config {
      public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake;

      public static final InvertedValue MotorInverted =
          InvertedValue.CounterClockwise_Positive; // Motor inverstion type (CW+ / CCW+)
      public static final double CurrentLimit = 40.0; // Max current in Amperes
    }
  }

  public class AlgaeSubsystem {
    public static final int IntakeMotorId = 31;
    public static final int PrimaryArmMotorId = 0; // The one connected to the elevator
    public static final int SecondaryArmMotorId = 0; // The one connected to the
    // pivot of the primary arm

    public static final int PrimaryArmGyroId = 0;
    public static final int SecondaryArmCANCoderId = 0;

    public static final int CANRangeId = 0; // Distance sensor
    public static final double hasAlgaeThreshold = 0.0; // The value where the sensor sees an algae

    public static final double IntakeSpeed = 0.3; // Default Intake Speed
    public static final double IntakeHoldbackSpeed =
        0.08; // Intake speed when holding an algae (This is to hold the algae in place).

    public static final double OuttakeSpeed = -0.5; // Default Outtake Speed (Negative value)

    public class PrimaryArm {
      public class PIDFF {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;
      }

      public class Angles {
        // TODO:: COMPLETE THIS
      }
    }

    public class SecondaryArm {
      public class PIDFF {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;
      }

      public class Angles {
        // TODO:: COMPLETE THIS
      }
    }

    public class Config {
      public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake;

      public static final InvertedValue MotorInverted =
          InvertedValue.CounterClockwise_Positive; // Motor inverstion type (CW+ / CCW+)
      public static final double CurrentLimit = 40.0; // Max current in Amperes
    }
  }

  public class ElevatorSubsystem {
    public static final int TopMotorId = 32;
    public static final int BottomMotorId = 33;

    public static final double AscendMaxSpeed = 0.5;
    public static final double DescendMaxSpeed = -0.2; // (Negative value)

    public class PIDFF {
      public static final double kP = 0.05; // 0.1;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double kS = 0;
      public static final double kG =
          0.738; // Manually calculated (TODO:: RECALCULATE TO NEW ELEVATOR, THE WEIGHT+GEAR RATIO
      // CHANGED)
      public static final double kV = 0;
      public static final double kA = 0;
    }

    public class Setpoints {
      public static final double MaxHeight = 77; // based on the motor sensor
      public static final double MinimumHeight = -1.0; // based on the motor sensor

      public static final double L0 = -1.0; // -1.0 is basically 0
      public static final double L1 = -1.0; // -1.0 is basically 0
      public static final double L2 = 22.5;
      public static final double L3 = 0.0; // not tested yet

      public static final double AlgaeScoringHeight = 0.0; // not tested yet (probably MaxHeight)
      public static final double ProcessorScoringHeight =
          0.0; // not tested yet (probably not needed since we have arms for algae)
    }

    public class Config {
      public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake;

      public static final InvertedValue MotorInverted =
          InvertedValue.CounterClockwise_Positive; // Motor inversion type (CW+ / CCW+)
      public static final double CurrentLimit = 40.0; // Max current in Amperes
    }
  }

  public class LED {
    public static final int pwmPort = 0;
    public static final int ledCount = 0;
  }
}
