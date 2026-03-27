package frc.robot.subsystems.Drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.util.LoggedTunableNumber;

/** Aim helpers: virtual target poses + driver align commands. */
public class AutoAim {
    private final CommandSwerveDrivetrain drive;
    private final RobotContainer robotContainer;

    private static final InterpolatingDoubleTreeMap kVirtualLeadMap = new InterpolatingDoubleTreeMap();
    private static final LoggedTunableNumber kLead0 = new LoggedTunableNumber("Drive/VirtualLead/0.0", 0.0, true);
    private static final LoggedTunableNumber kLead1 = new LoggedTunableNumber("Drive/VirtualLead/1.0", -1.4, true);
    private static final LoggedTunableNumber kLead2 = new LoggedTunableNumber("Drive/VirtualLead/2.0", -2.0, true);
    private static final LoggedTunableNumber kLead3 = new LoggedTunableNumber("Drive/VirtualLead/3.0", -2.8, true);
    private static final LoggedTunableNumber kLead4 = new LoggedTunableNumber("Drive/VirtualLead/4.0", -2.7, true);

    private static double getAppliedVirtualLead(ChassisSpeeds speeds) {
        // dashboard wins if someone typed a value
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

    /** Extra NT for AdvantageKit / logging (ferry still uses this; hub pose also on Pose/hubVirtualPose). */
    private static void publishVirtualPoseNt(String ntSuffix, Translation2d trans, Pose2d referencePose) {
        var nt = NetworkTableInstance.getDefault();
        String p = "AdvantageKit/Drive/" + ntSuffix;
        nt.getEntry(p + "/PoseX").setDouble(trans.getX());
        nt.getEntry(p + "/PoseY").setDouble(trans.getY());
        nt.getEntry(p + "/PoseHeadingDeg").setDouble(referencePose.getRotation().getDegrees());
    }

    private static Pose2d virtualPoseWithLead(
            Pose2d robotPose, ChassisSpeeds speeds, Pose2d basePose, String ntSuffix) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double speed = Math.hypot(vx, vy);
        double lead = getAppliedVirtualLead(speeds);

        if (speed <= 1e-6 || lead == 0.0) {
            return basePose;
        }

        Translation2d robotRel = new Translation2d(vx, vy);
        Translation2d fieldRel = robotRel.rotateBy(robotPose.getRotation());
        Translation2d direction = fieldRel.div(speed);

        Translation2d resultTrans = basePose.getTranslation().plus(direction.times(lead));
        publishVirtualPoseNt(ntSuffix, resultTrans, basePose);
        return new Pose2d(resultTrans, basePose.getRotation());
    }

    public static Pose2d getVirtualHubPose(Pose2d robotPose, ChassisSpeeds speeds) {
        return virtualPoseWithLead(robotPose, speeds, DriveConstants.getHubPose(), "VirtualHub");
    }

    public static Pose2d getVirtualLeftFerryPose(Pose2d robotPose, ChassisSpeeds speeds) {
        return virtualPoseWithLead(robotPose, speeds, DriveConstants.getLeftFerryPose(), "VirtualLeftFerry");
    }

    public static Pose2d getVirtualRightFerryPose(Pose2d robotPose, ChassisSpeeds speeds) {
        return virtualPoseWithLead(robotPose, speeds, DriveConstants.getRightFerryPose(), "VirtualRightFerry");
    }

    public AutoAim(CommandSwerveDrivetrain drive, RobotContainer robotContainer) {
        this.drive = drive;
        this.robotContainer = robotContainer;
    }

    private Command prepShot(Supplier<Pose2d> targetPose, boolean isHub) {
        return Commands.parallel(drive.alignDrive(robotContainer.driver, targetPose, isHub));
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
