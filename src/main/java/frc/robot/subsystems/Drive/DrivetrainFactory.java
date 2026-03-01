package frc.robot.subsystems.Drive;

/**
 * Non-generated factory to construct the application-visible drivetrain wrapper.
 *
 * Rationale: Keep generated CTRE files untouched and provide a single place to
 * construct the app-facing `SwerveDrivetrain` wrapper. Use this factory from
 * `RobotContainer` to centralize creation.
 */
public final class DrivetrainFactory {
    private DrivetrainFactory() {}

    public static SwerveDrivetrain create() {
        return new SwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        );
    }
}
