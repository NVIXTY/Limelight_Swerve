package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Thin wrapper left in place so code that imports `SwerveDrivetrain` keeps working.
 * The heavy implementation lives in `CommandSwerveDrivetrain` (restored from generated code).
 */
public class SwerveDrivetrain extends CommandSwerveDrivetrain {
    public SwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
    }

    public SwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
    }

    public SwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    }
}

