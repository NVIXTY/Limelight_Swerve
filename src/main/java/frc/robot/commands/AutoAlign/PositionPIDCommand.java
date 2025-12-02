// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign  ;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightGlobalPose.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

@SuppressWarnings("LocalVariableName")
public class PositionPIDCommand {
    private final PIDController m_xyController = new PIDController(0, 0, 0);
    private final PIDController m_zController = new PIDController(0, 0, 0);
    private Pose2d m_poseError;

    @SuppressWarnings("LocalVariableName")
    public ChassisSpeeds calculate(
        Pose2d currentPose,
        Pose2d poseRef) {
        // Update PID constants from Constants.DriveConstants
        m_xyController.setPID(
            Constants.DriveConstants.kHolonomicXYP,
            Constants.DriveConstants.kHolonomicXYI,
            Constants.DriveConstants.kHolonomicXYD
        );
        m_zController.setPID(
            Constants.DriveConstants.kHolonomicThetaP,
            Constants.DriveConstants.kHolonomicThetaI,
            Constants.DriveConstants.kHolonomicThetaD
        );

        m_poseError = poseRef.relativeTo(currentPose);

        // Calculate feedback velocities (based on position error).
        double x = m_xyController.calculate(currentPose.getX(), poseRef.getX());
        double y = m_xyController.calculate(currentPose.getY(), poseRef.getY());
        double thetaFeedback = m_zController.calculate(
            currentPose.getRotation().getDegrees(),
            poseRef.getRotation().getDegrees()
        );

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            x * Constants.DriveConstants.kMaxSpeedMetersPerSecond,
            y * Constants.DriveConstants.kMaxSpeedMetersPerSecond,
            thetaFeedback * Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecond,
            currentPose.getRotation()
        );
    }
}