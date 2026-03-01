package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive.DriveConstants;

public class PrepRightFerryShot extends SequentialCommandGroup {
    public PrepRightFerryShot(SwerveDrivetrain drivetrain, CommandXboxController driver) {
        super(drivetrain.alignDrive(driver, () -> DriveConstants.getRightFerryPose().toPose2d()));
    }
}
