package frc.robot.subsystems.Drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.util.ShotCalculator;

/** Aim helpers: virtual target poses + driver align commands. */
public class AutoAim {
    private final CommandSwerveDrivetrain drive;
    private final RobotContainer robotContainer;

    public AutoAim(CommandSwerveDrivetrain drive, RobotContainer robotContainer) {
        this.drive = drive;
        this.robotContainer = robotContainer;
    }

    /**
     * Get the velocity-compensated virtual hub pose using ShotCalculator.
     * @return Compensated virtual hub pose
     */
    public static Pose2d getVirtualHubPose() {
        return ShotCalculator.getInstance().getVirtualTarget(DriveConstants.getHubPose());
    }

    /**
     * Get the velocity-compensated virtual left ferry pose using ShotCalculator.
     * @return Compensated virtual left ferry pose
     */
    public static Pose2d getVirtualLeftFerryPose() {
        return ShotCalculator.getInstance().getVirtualTarget(DriveConstants.getLeftFerryPose());
    }

    /**
     * Get the velocity-compensated virtual right ferry pose using ShotCalculator.
     * @return Compensated virtual right ferry pose
     */
    public static Pose2d getVirtualRightFerryPose() {
        return ShotCalculator.getInstance().getVirtualTarget(DriveConstants.getRightFerryPose());
    }

    private Command prepShot(Supplier<Pose2d> targetPose, boolean isHub) {
        return Commands.parallel(drive.alignDrive(robotContainer.driver, targetPose, isHub));
    }

    public Command prepHubShot() {
        return prepShot(AutoAim::getVirtualHubPose, true);
    }

    public Command prepLeftFerryShot() {
        return prepShot(AutoAim::getVirtualLeftFerryPose, false);
    }

    public Command prepRightFerryShot() {
        return prepShot(AutoAim::getVirtualRightFerryPose, false);
    }
}
