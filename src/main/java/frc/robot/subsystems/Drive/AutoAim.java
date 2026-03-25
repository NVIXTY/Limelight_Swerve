package frc.robot.subsystems.Drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutoAim {
    private final CommandSwerveDrivetrain drive;
    private final RobotContainer robotContainer;

    // Virtual lead tuning map moved here from DriveConstants so auto-align
    // logic lives in Superstructure and DriveConstants remains purely constants.
    private static InterpolatingDoubleTreeMap kVirtualLeadMap = new InterpolatingDoubleTreeMap();
    private static final LoggedTunableNumber kLead0 = new LoggedTunableNumber("Drive/VirtualLead/0.0", 0.0, true);
    private static final LoggedTunableNumber kLead1 = new LoggedTunableNumber("Drive/VirtualLead/1.0", -1.4, true);
    private static final LoggedTunableNumber kLead2 = new LoggedTunableNumber("Drive/VirtualLead/2.0", -2.0, true);
    private static final LoggedTunableNumber kLead3 = new LoggedTunableNumber("Drive/VirtualLead/3.0", -2.8, true);
    private static final LoggedTunableNumber kLead4 = new LoggedTunableNumber("Drive/VirtualLead/4.0", -2.7, true);

    /**
     * Helper to compute the applied virtual lead for the current chassis speeds.
     * Both hub and ferry virtual methods call this so they use the exact tuned
     * values.
     */
    private static double getAppliedVirtualLead(ChassisSpeeds speeds) {
        double l0 = SmartDashboard.getNumber("Drive/VirtualLead/0.0", kLead0.get());
        double l1 = SmartDashboard.getNumber("Drive/VirtualLead/1.0", kLead1.get());
        double l2 = SmartDashboard.getNumber("Drive/VirtualLead/2.0", kLead2.get());
        double l3 = SmartDashboard.getNumber("Drive/VirtualLead/3.0", kLead3.get());
        double l4 = SmartDashboard.getNumber("Drive/VirtualLead/4.0", kLead4.get());

        kVirtualLeadMap.put(0.0, l0);
        kVirtualLeadMap.put(1.0, l1);
        kVirtualLeadMap.put(2.0, l2);
        kVirtualLeadMap.put(3.0, l3);
        kVirtualLeadMap.put(4.0, l4);

        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double speed = Math.hypot(vx, vy);
        double lead = kVirtualLeadMap.get(speed);

        Logger.recordOutput("Drive/VirtualLead/AppliedLead", lead);
        Logger.recordOutput("Drive/VirtualLead/AppliedSpeed", speed);

        return lead;
    }

    public static Pose2d getVirtualHubPose(Pose2d robotPose, ChassisSpeeds speeds) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double speed = Math.hypot(vx, vy);
        double lead = getAppliedVirtualLead(speeds);

        if (speed <= 1e-6 || lead == 0.0) {
            return DriveConstants.getHubPose();
        }

        Translation2d robotRel = new Translation2d(vx, vy);
        Translation2d fieldRel = robotRel.rotateBy(robotPose.getRotation());
        Translation2d direction = fieldRel.div(speed);

        Translation2d hubTrans = DriveConstants.getHubPose().getTranslation().plus(direction.times(lead));
        var nt = NetworkTableInstance.getDefault();
        nt.getEntry("AdvantageKit/Drive/VirtualHub/PoseX").setDouble(hubTrans.getX());
        nt.getEntry("AdvantageKit/Drive/VirtualHub/PoseY").setDouble(hubTrans.getY());
        nt.getEntry("AdvantageKit/Drive/VirtualHub/PoseHeadingDeg").setDouble(DriveConstants.getHubPose().getRotation().getDegrees());

        return new Pose2d(hubTrans, DriveConstants.getHubPose().getRotation());
    }

    public static Pose2d getVirtualLeftFerryPose(Pose2d robotPose, ChassisSpeeds speeds) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double speed = Math.hypot(vx, vy);
        double lead = getAppliedVirtualLead(speeds);

        if (speed <= 1e-6 || lead == 0.0) {
            return DriveConstants.getLeftFerryPose();
        }

        Translation2d robotRel = new Translation2d(vx, vy);
        Translation2d fieldRel = robotRel.rotateBy(robotPose.getRotation());
        Translation2d direction = fieldRel.div(speed);

        Translation2d ferryTrans = DriveConstants.getLeftFerryPose().getTranslation().plus(direction.times(lead));
        var nt = NetworkTableInstance.getDefault();
        nt.getEntry("AdvantageKit/Drive/VirtualLeftFerry/PoseX").setDouble(ferryTrans.getX());
        nt.getEntry("AdvantageKit/Drive/VirtualLeftFerry/PoseY").setDouble(ferryTrans.getY());
        nt.getEntry("AdvantageKit/Drive/VirtualLeftFerry/PoseHeadingDeg").setDouble(DriveConstants.getLeftFerryPose().getRotation().getDegrees());

        return new Pose2d(ferryTrans, DriveConstants.getLeftFerryPose().getRotation());
    }

    public static Pose2d getVirtualRightFerryPose(Pose2d robotPose, ChassisSpeeds speeds) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double speed = Math.hypot(vx, vy);
        double lead = getAppliedVirtualLead(speeds);

        if (speed <= 1e-6 || lead == 0.0) {
            return DriveConstants.getRightFerryPose();
        }

        Translation2d robotRel = new Translation2d(vx, vy);
        Translation2d fieldRel = robotRel.rotateBy(robotPose.getRotation());
        Translation2d direction = fieldRel.div(speed);

        Translation2d ferryTrans = DriveConstants.getRightFerryPose().getTranslation().plus(direction.times(lead));
        var nt = NetworkTableInstance.getDefault();
        nt.getEntry("AdvantageKit/Drive/VirtualRightFerry/PoseX").setDouble(ferryTrans.getX());
        nt.getEntry("AdvantageKit/Drive/VirtualRightFerry/PoseY").setDouble(ferryTrans.getY());
        nt.getEntry("AdvantageKit/Drive/VirtualRightFerry/PoseHeadingDeg").setDouble(DriveConstants.getRightFerryPose().getRotation().getDegrees());

        return new Pose2d(ferryTrans, DriveConstants.getRightFerryPose().getRotation());
    }

    public AutoAim(CommandSwerveDrivetrain drive, RobotContainer robotContainer) {
        this.drive = drive;
        this.robotContainer = robotContainer;
    }

    private Command prepShot(Supplier<Pose2d> targetPose, boolean isHub) {
        return Commands.parallel(
                drive.alignDrive(robotContainer.driver, targetPose, isHub)
        );
    }

    public Command prepHubShot() {
        return prepShot(() -> getVirtualHubPose(drive.getState().Pose, drive.getState().Speeds), true);
    }

    public Command prepLeftFerryShot() {
        return prepShot(() -> getVirtualLeftFerryPose(drive.getState().Pose, drive.getState().Speeds), false);
    }

    public Command prepRightFerryShot() {
        return prepShot(() -> getVirtualRightFerryPose(drive.getState().Pose, drive.getState().Speeds), false);
    }
}