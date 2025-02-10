package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class RobotConstants {
    public class Controllers {
        public static final int DriverPortId = 0;
        public static final int OperatorPortId = 1;

        // public static final int DebugPortId = 5; // Future use
    }

    public class SwerveSettings{
        public static final double baseSpeedPercentage = 0.7; // Base speed without modifications; (X * 100)% of the MAXIMUM speed (1.0)

        public static final double maximumSpeedPercentage = 1.0; // Keep at 1.0 for absolute maximum speed (100%)
        public static final double minimumSpeedPercentage = 0.3; // (X * 100)% of the MAXIMUM speed (100%)
    }

    public class CoralSubsystem {
        public static final int IntakeMotorId = 30;

        public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake;

        public static final double IntakeSpeed = 0.3; // Default Intake Speed
        public static final double OuttakeSpeed = -0.3; // Default Outtake Speed (Negative value)
    }

    public class AlgaeSubsystem {
        public static final int IntakeMotorId = 31;

        public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake;

        public static final double IntakeSpeed = 0.3; // Default Intake Speed
        public static final double IntakeHoldbackSpeed = 0.08; // Intake speed when holding an algae (This is to hold the algae in place).

        public static final double OuttakeSpeed = -0.5; // Default Outtake Speed (Negative value)
    }
}
