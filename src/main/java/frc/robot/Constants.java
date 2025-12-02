package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Constants {
    public static final class DriveConstants {
        // PID Constants for Holonomic Drive
        public static final double kHolonomicXYP = 1.0;
        public static final double kHolonomicXYI = 0.0;
        public static final double kHolonomicXYD = 0.0;

        public static final double kHolonomicThetaP = 1.0;
        public static final double kHolonomicThetaI = 0.0;
        public static final double kHolonomicThetaD = 0.0;

        // Maximum Speed Constants
        public static final double kMaxSpeedMetersPerSecond = 3.0; // Example value, adjust as needed
        public static final double kMaxAngularSpeedRadiansPerSecond = Units.degreesToRadians(180.0); // Example value, adjust as needed
    }
}