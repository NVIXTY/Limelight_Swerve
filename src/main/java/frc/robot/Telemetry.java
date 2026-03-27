package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import frc.robot.subsystems.Drive.AutoAim;

/**
 * Swerve + field NT. Hub virtual pose (/Pose/hubVirtualPose) only updates while
 * {@code Pose/PublishHubVirtualPose} is true (right trigger hub align in RobotContainer).
 * Bind Elastic overlay visibility to {@code Pose/hubVirtualPoseActive} if you want it hidden otherwise.
 */
public class Telemetry {
    private final double MaxSpeed;

    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();

        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose =
        driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds =
        driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates =
        driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets =
        driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions =
        driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency =
        driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    // same pattern WPILib uses for Field2d → Elastic can bind these
    private final NetworkTable poseTable = inst.getTable("Pose");
    private final DoubleArrayPublisher robotPoseField = poseTable.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher robotPoseFieldType = poseTable.getStringTopic(".type").publish();

    private final DoubleArrayPublisher hubVirtualPoseField =
        poseTable.getDoubleArrayTopic("hubVirtualPose").publish();
    private final StringPublisher hubVirtualPoseFieldType =
        poseTable.getStringTopic("hubVirtualPose/.type").publish();
    private final BooleanPublisher hubVirtualPoseActive =
        poseTable.getBooleanTopic("hubVirtualPoseActive").publish();

    private final Mechanism2d[] m_moduleMechanisms = {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    private final MechanismLigament2d[] m_moduleSpeeds = {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    private final MechanismLigament2d[] m_moduleDirections = {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] robotPoseTriplet = new double[3];
    private final double[] hubVirtualTriplet = new double[3];

    public void telemeterize(SwerveDriveState state) {
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        robotPoseFieldType.set("Field2d");
        robotPoseTriplet[0] = state.Pose.getX();
        robotPoseTriplet[1] = state.Pose.getY();
        robotPoseTriplet[2] = state.Pose.getRotation().getDegrees();
        robotPoseField.set(robotPoseTriplet);

        boolean showHubVirt = SmartDashboard.getBoolean("Pose/PublishHubVirtualPose", false);
        hubVirtualPoseActive.set(showHubVirt);
        if (showHubVirt) {
            Pose2d hubVirt = AutoAim.getVirtualHubPose(state.Pose, state.Speeds);
            hubVirtualPoseFieldType.set("Field2d");
            hubVirtualTriplet[0] = hubVirt.getX();
            hubVirtualTriplet[1] = hubVirt.getY();
            hubVirtualTriplet[2] = hubVirt.getRotation().getDegrees();
            hubVirtualPoseField.set(hubVirtualTriplet);
        }

        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }
    }
}
