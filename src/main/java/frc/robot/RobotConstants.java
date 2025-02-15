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
        0.5; // Base speed without modifications; (X * 100)% of the MAXIMUM speed (1.0)

    public static final double maximumSpeedPercentage =
        1.0; // Keep at 1.0 for absolute maximum speed (100%)
    public static final double minimumSpeedPercentage =
        0.05; // (X * 100)% of the MAXIMUM speed (100%)
  }

  public class CoralSubsystem {
    public static final int IntakeMotorId = 30;

    public static final double IntakeSpeed = 0.3; // Default Intake Speed
    public static final double OuttakeSpeed = -0.3; // Default Outtake Speed (Negative value)

    public class AngleSystem {
      public static final int MotorId = 34;

      public class PIDFF {
        public static final double kP = 0.08;
        public static final double kI = 0;
        public static final double kD = 0.003; // 0.005;

        public static final double kS = 0;
        public static final double kG = 0; // Manually calculated
        public static final double kV = 0;
        public static final double kA = 0;
      }
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

    public static final double IntakeSpeed = 0.3; // Default Intake Speed
    public static final double IntakeHoldbackSpeed =
        0.08; // Intake speed when holding an algae (This is to hold the algae in place).

    public static final double OuttakeSpeed = -0.5; // Default Outtake Speed (Negative value)

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

    public static final double MotorSpeed = 0.5; // temporary

    public class PIDFF {
      public static final double kP = 0.05; // 0.1;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double kS = 0;
      public static final double kG = 0.738; // Manually calculated
      public static final double kV = 0;
      public static final double kA = 0;
    }

    public class Config {
      public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake;

      public static final InvertedValue MotorInverted =
          InvertedValue.CounterClockwise_Positive; // Motor inversion type (CW+ / CCW+)
      public static final double CurrentLimit = 40.0; // Max current in Amperes
    }
  }
}
