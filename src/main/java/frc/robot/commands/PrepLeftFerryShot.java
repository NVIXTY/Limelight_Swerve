package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive.DriveConstants;

public class PrepLeftFerryShot extends SequentialCommandGroup {
    public PrepLeftFerryShot(SwerveDrivetrain drivetrain, CommandXboxController driver) {
        super(drivetrain.alignDrive(driver, () -> DriveConstants.getLeftFerryPose().toPose2d()));
    }
}
