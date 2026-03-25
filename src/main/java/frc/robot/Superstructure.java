package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.DriveConstants;

public class Superstructure {
    private final CommandSwerveDrivetrain drive;
    private final RobotContainer robotContainer;

    public Superstructure(CommandSwerveDrivetrain drive, RobotContainer robotContainer) {
        this.drive = drive;
        this.robotContainer = robotContainer;
    }

    private Command prepShot(Supplier<Pose2d> targetPose) {
        return Commands.parallel(
                drive.alignDrive(robotContainer.driver, targetPose)
        );
    }

    public Command prepHubShot() {
        return prepShot(() -> DriveConstants.getVirtualHubPose(drive.getState().Pose, drive.getState().Speeds));
    }

    public Command prepLeftFerryShot() {
        return prepShot(() -> DriveConstants.getLeftFerryPose());
    }

    public Command prepRightFerryShot() {
        return prepShot(() -> DriveConstants.getRightFerryPose());
    }
}