package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LoggedTunableNumber;

/** Field poses + align tuning. Flip handled for blue. */
public class DriveConstants {
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
    public static final double maxAngularRate = Units.RotationsPerSecond.of(1).in(Units.RadiansPerSecond);

    public static final Distance shooterSideOffset = Units.Inches.of(0);

    public static final Transform2d shooterTransform =
        new Transform2d(Units.Inches.of(0.0), shooterSideOffset, new Rotation2d());

    public static final Pose2d redHubPose = new Pose2d(11.921, 4.027, Rotation2d.fromDegrees(0));
    public static final Pose2d redLeftFerryPose = new Pose2d(16, 1, Rotation2d.fromDegrees(0));
    public static final Pose2d redRightFerryPose = new Pose2d(16, 7, Rotation2d.fromDegrees(0));
    public static final Pose2d blueHubPose = FlippingUtil.flipFieldPose(redHubPose);
    public static final Pose2d blueLeftFerryPose = FlippingUtil.flipFieldPose(redLeftFerryPose);
    public static final Pose2d blueRightFerryPose = FlippingUtil.flipFieldPose(redRightFerryPose);

    public static final double stickDeadband = 0.1;

    public static final Angle epsilonAngleToGoal = Degrees.of(0);

    private static final LoggedTunableNumber kHubLateralScale =
        new LoggedTunableNumber("Drive/Hub/LateralScale", 0.2, true);

    public static double getHubLateralScale() {
        return SmartDashboard.getNumber("Drive/Hub/LateralScale", kHubLateralScale.get());
    }

    public static Pose2d getHubPose() {
        return DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
    }

    public static Pose2d getLeftFerryPose() {
        return DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redLeftFerryPose : blueLeftFerryPose;
    }

    public static Pose2d getRightFerryPose() {
        return DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redRightFerryPose : blueRightFerryPose;
    }

    public static final PIDController rotationController = createRotationController();

    private static PIDController createRotationController() {
        PIDController controller = new PIDController(1.4, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        return controller;
    }
}
