package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
        new LoggedTunableNumber("Drive/Hub/LateralScale", 0.225, true);

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

    // Air time lookup table for SOTM compensation - maps distance (meters) to air time (seconds)
    // Tune these values based on your shooter's projectile speed
    private static final InterpolatingDoubleTreeMap kAirTimeMap = new InterpolatingDoubleTreeMap();
    private static final LoggedTunableNumber kAirTime1m = new LoggedTunableNumber("Drive/SOTM/AirTime/1m", 1.2, true);
    private static final LoggedTunableNumber kAirTime2m = new LoggedTunableNumber("Drive/SOTM/AirTime/2m", 1, true);
    private static final LoggedTunableNumber kAirTime3m = new LoggedTunableNumber("Drive/SOTM/AirTime/3m", 1.15, true);
    private static final LoggedTunableNumber kAirTime4m = new LoggedTunableNumber("Drive/SOTM/AirTime/4m", 1.2, true);
    private static final LoggedTunableNumber kAirTime5m = new LoggedTunableNumber("Drive/SOTM/AirTime/5m", 1.25, true);
    private static final LoggedTunableNumber kAirTime6m = new LoggedTunableNumber("Drive/SOTM/AirTime/6m", 1.35, true);

    /**
     * Get air time for a given distance using interpolation map.
     * @param distanceMeters Distance to target in meters
     * @return Estimated air time in seconds
     */
    public static double getAirTime(double distanceMeters) {
        // Update map with current tunable values each call
        kAirTimeMap.put(1.0, kAirTime1m.get());
        kAirTimeMap.put(2.0, kAirTime2m.get());
        kAirTimeMap.put(3.0, kAirTime3m.get());
        kAirTimeMap.put(4.0, kAirTime4m.get());
        kAirTimeMap.put(5.0, kAirTime5m.get());
        kAirTimeMap.put(6.0, kAirTime6m.get());
        return kAirTimeMap.get(distanceMeters);
    }
}
