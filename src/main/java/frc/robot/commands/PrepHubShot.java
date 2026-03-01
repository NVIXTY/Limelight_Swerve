package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive.DriveConstants;

/**
 * Prep a hub shot by running the drivetrain's alignDrive command.
 * Uses WPILib Command composition instead of a team-specific coordinator.
 */
public class PrepHubShot extends SequentialCommandGroup {
    public PrepHubShot(SwerveDrivetrain drivetrain, CommandXboxController driver) {
        super(drivetrain.alignDrive(driver, () -> DriveConstants.getHubPose().toPose2d()));
    }
}
