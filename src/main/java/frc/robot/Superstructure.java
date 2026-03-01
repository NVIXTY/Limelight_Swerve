package frc.robot;

import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.DriveConstants;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class Superstructure {
    public final CommandSwerveDrivetrain drive;
    private final RobotContainer robotContainer;


    public Superstructure(CommandSwerveDrivetrain drive, RobotContainer robotContainer) {
        this.drive = drive;
        this.robotContainer = robotContainer;}
        // NamedCommands.getInstance() is not available in this PathPlanner version.
        // If you need to register a named command, use the appropriate API for your PathPlanner release.
        // For now, the registration is commented out to allow compilation.
        // NamedCommands.getInstance().addNamedCommand("Align Hub", prepHubShot());

    private Command prepShot(Supplier<Pose2d> targetPose) {
        return Commands.parallel(
            drive.alignDrive(robotContainer.driver, targetPose)
        );
    }
        ;
    

    public Command prepRightFerryShot() {
        return prepShot(() -> DriveConstants.getRightFerryPose().toPose2d());
    } 

    public Command prepLeftFerryShot() {
        return prepShot(() -> DriveConstants.getLeftFerryPose().toPose2d());
    } 

    public Command prepHubShot() {
        return prepShot(() -> DriveConstants.getHubPose().toPose2d());
    }
}