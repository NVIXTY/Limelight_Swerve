package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriveConstants {
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double maxAngularRate = Units.RotationsPerSecond.of(1).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    public static final Distance shooterSideOffset = Units.Inches.of(0);

    public static final Transform2d shooterTransform = new Transform2d(Units.Inches.of(0.0), shooterSideOffset, new Rotation2d());

    public static final Pose2d redHubPose = new Pose2d(11.921, 4.027, Rotation2d.fromDegrees(0));
    public static final Pose2d redLeftFerryPose = new Pose2d(16, .55, Rotation2d.fromDegrees(0));
    public static final Pose2d redRightFerryPose = new Pose2d(16, 7.55, Rotation2d.fromDegrees(0));
    public static final Pose2d blueHubPose = FlippingUtil.flipFieldPose(redHubPose);
    public static final Pose2d blueLeftFerryPose = FlippingUtil.flipFieldPose(redLeftFerryPose);
    public static final Pose2d blueRightFerryPose = FlippingUtil.flipFieldPose(redRightFerryPose);

    public static final double stickDeadband = 0.1;
    

    public static final Angle epsilonAngleToGoal = Degrees.of(0);

    public static final Pose2d getHubPose() {
        Pose2d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
        return pose;
    }

    // Virtual hub pose tuning map: speed (m/s) -> lead (m)
    public static InterpolatingDoubleTreeMap kVirtualLeadMap = new InterpolatingDoubleTreeMap();

    private static final LoggedTunableNumber kLead0 = new LoggedTunableNumber("Drive/VirtualLead/0.0", 0.0, true);
    private static final LoggedTunableNumber kLead1 = new LoggedTunableNumber("Drive/VirtualLead/1.0", -1.4, true);
    private static final LoggedTunableNumber kLead2 = new LoggedTunableNumber("Drive/VirtualLead/2.0", -2.0, true);
    private static final LoggedTunableNumber kLead3 = new LoggedTunableNumber("Drive/VirtualLead/3.0", -2.8, true);
    private static final LoggedTunableNumber kLead4 = new LoggedTunableNumber("Drive/VirtualLead/4.0", -2.7, true);

    /**
     * Returns a virtual hub pose adjusted by a small lead amount in the direction
     * of the robot's translational velocity. The lead distance is looked up from
     * an interpolation map keyed by robot speed (m/s) and is tunable via
     * SmartDashboard/LoggedTunableNumbers.
     *
     * @param robotPose current robot pose (field coordinates)
     * @param speeds current robot chassis speeds (robot-relative vx, vy in m/s)
     * @return adjusted Pose2d representing the virtual hub pose
     */
    public static Pose2d getVirtualHubPose(Pose2d robotPose, ChassisSpeeds speeds) {
    // Read current tunables (SmartDashboard overrides LoggedTunableNumber)
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

    // Record applied values for diagnostics
    Logger.recordOutput("Drive/VirtualLead/AppliedLead", lead);
    Logger.recordOutput("Drive/VirtualLead/AppliedSpeed", speed);

    // Publish a compact virtual-hub pose to NetworkTables for AdvantageKit or other NT consumers.
    // (Actual pose values are written below after hub translation is computed.)

        if (speed <= 1e-6 || lead == 0.0) {
            return getHubPose();
        }

        // Robot-relative velocity -> field-relative translation
        Translation2d robotRel = new Translation2d(vx, vy);
        Translation2d fieldRel = robotRel.rotateBy(robotPose.getRotation());
        Translation2d direction = fieldRel.div(speed); // normalized field-relative direction

        Translation2d hubTrans = getHubPose().getTranslation().plus(direction.times(lead));
    // Publish the virtual hub pose to NetworkTables for AdvantageKit/NT consumers.
    var nt = NetworkTableInstance.getDefault();
    nt.getEntry("AdvantageKit/Drive/VirtualHub/PoseX").setDouble(hubTrans.getX());
    nt.getEntry("AdvantageKit/Drive/VirtualHub/PoseY").setDouble(hubTrans.getY());
    nt.getEntry("AdvantageKit/Drive/VirtualHub/PoseHeadingDeg").setDouble(getHubPose().getRotation().getDegrees());

    return new Pose2d(hubTrans, getHubPose().getRotation());
    }

    public static final Pose2d getLeftFerryPose() {
        Pose2d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redLeftFerryPose : blueLeftFerryPose;
        return pose;
    }

    public static final Pose2d getRightFerryPose() {
        Pose2d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redRightFerryPose : blueRightFerryPose;
        return pose;
    }

    public static final PIDController rotationController = getRotationController();

    private static final PIDController getRotationController() {
        PIDController controller = new PIDController(1.4, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        return controller;
    }
    
}