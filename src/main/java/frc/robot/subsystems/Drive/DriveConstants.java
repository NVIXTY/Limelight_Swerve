package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriveConstants {
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double maxAngularRate = Units.RotationsPerSecond.of(1).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    public static final Distance shooterSideOffset = Units.Inches.of(0);

    public static final Transform2d shooterTransform = new Transform2d(Units.Inches.of(0.0), shooterSideOffset, new Rotation2d());

    public static final Pose3d redHubPose = new Pose3d(5, 4, 0, new Rotation3d());
    public static final Pose3d redLeftFerryPose = new Pose3d(14.3, 4.02, 0, Rotation3d.kZero);
    public static final Pose3d redRightFerryPose = new Pose3d(2.5, 6.5, 0, Rotation3d.kZero);
    public static final Pose3d blueHubPose = new Pose3d(5, 4, 0, new Rotation3d());
    public static final Pose3d blueLeftFerryPose = new Pose3d(14.3, 4.02, 0, Rotation3d.kZero);
    public static final Pose3d blueRightFerryPose = new Pose3d(2.5, 6.5, 0, Rotation3d.kZero);

    public static final double stickDeadband = 0.1;
    

    public static final Angle epsilonAngleToGoal = Degrees.of(-1.0);

    public static final Pose3d getHubPose() {
        Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
        return pose;
    }

    public static final Pose3d getLeftFerryPose() {
        Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redLeftFerryPose : blueLeftFerryPose;
        return pose;
    }

    public static final Pose3d getRightFerryPose() {
        Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redRightFerryPose : blueRightFerryPose;
        return pose;
    }

    public static final PIDController rotationController = getRotationController();

    private static final PIDController getRotationController() {
        PIDController controller = new PIDController(2.0, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        return controller;
    }
    
}
