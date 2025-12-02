// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;

import static edu.wpi.first.units.Units.*;


import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LimelightGlobalPose.CommandSwerveDrivetrain;

/**
 * The AlignToReef command is responsible for aligning the robot to a specific reef position
 * on the field. It utilizes the SwerveSubsystem for movement, the ElevatorSubsystem for 
 * vertical positioning, and the EndEffectorSubsystem for end effector operations. The command 
 * supports aligning to predefined reef positions, adjusting the robot's heading based on 
 * velocity or target pose, and moving the elevator to a desired reef level.
 * 
 */

public class DriveToPose extends Command {
  /** Creates a new AlignToReef. */

  //Subsystems
  private CommandSwerveDrivetrain m_swerveSubsystem;

  private Pose2d m_targetPose = new Pose2d();

  private static SwerveRequest.ApplyRobotSpeeds m_swerveRequest = new ApplyRobotSpeeds();

  private boolean isAutoAdjustActive = false;

  


  public DriveToPose(CommandSwerveDrivetrain swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem = swerveSubsystem;

    addRequirements(m_swerveSubsystem);

  }
  /**
   * Method to generate a command to follow a path to a waypoint | 
   * Auto adjusts to the waypoint upon arrival | 
   * Auto adjusts override if within 0.25 meters of waypoint | 
   * Moves elevator to desired reef level if within 1 meter of waypoint | 
   * @param waypoint The target waypoint to align to
   * @return Command to follow the path to the waypoint
   */
  private Command getPathFromWaypoint(Pose2d waypoint) {
    // Create waypoints for pathplanner path
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(m_swerveSubsystem.getState().Pose.getTranslation(), getPathVelocityHeading(m_swerveSubsystem.getFieldVelocity(), waypoint)),
      waypoint
    );
    
    // Create path constraints
    PathConstraints pathConstraints = new PathConstraints(1, 1, 180, 360);

    // Create pathplanner path
    PathPlannerPath path = new PathPlannerPath(
                                              waypoints, 
                                              pathConstraints, 
                                              new IdealStartingState(getVelocityMagnitude(m_swerveSubsystem.getFieldVelocity()), m_swerveSubsystem.getState().Pose.getRotation()),
                                              new GoalEndState(0.0, waypoint.getRotation()));


    path.preventFlipping = true;
  
    m_targetPose = waypoint;

    // // Build and return command
    // return (AutoBuilder.followPath(path)
    //         .andThen(
    //                 // Auto Adjust after reaching the waypoint
    //                 PositionPIDCommand.generateCommand(m_swerveSubsystem, waypoint, Seconds.of(2),this)))


    //         .alongWith(
    //                 Commands.waitUntil(
    //                                   ()-> m_swerveSubsystem.getState().Pose.getTranslation().getDistance(waypoints.get(1).anchor()) < 1)
                  
    //                                   .andThen(
    //                                             // Auto adjust override if closer than 0.25 meters to waypoint
    //                                             PositionPIDCommand.generateCommand(m_swerveSubsystem, waypoint, Seconds.of(2),this)

    //                                             //Print Starting
    //                                             .beforeStarting(Commands.print("Starting final approach PID"))
    //                                             //Print Ending
    //                                             .andThen(Commands.print("Ending final approach PID"))
    //                                   ))
    //         // Until interrupted
    //         .finallyDo((Interupt) -> {
    //           if (Interupt) {
    //             // Stop the robot if interrupted
    //             m_swerveRequest.withSpeeds(new ChassisSpeeds(0, 0, 0));
    //           }
    // });
    return AutoBuilder.followPath(path);
  }
  


  /**
   * Method to get the velocity magnitude from chassis speeds 
   * @param cs ChassisSpeeds of the robot
   * @return LinearVelocity magnitude of the robot 
   */
  private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
    return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  
  }

  /**
   * Method to get the heading based on path velocity or target pose
   * @param cs ChassisSpeeds of the robot
   * @param targetPose Target Pose2d to align to
   * @return Rotation2d heading for the path 
   */
  private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d targetPose){
    if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) { // If the robot is moving slower than 0.25 m/s, face the target
      var diff = targetPose.minus(m_swerveSubsystem.getState().Pose).getTranslation();
      return (diff.getNorm() < 0.01) ? targetPose.getRotation() : diff.getAngle(); // If the robot is within 1 cm of the target, keep the target rotation
    }
    return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
  }




  public Command driveToPose(Pose2d targetPose){
    return Commands.defer(()-> {
      return getPathFromWaypoint(targetPose);
    }, Set.of()); 
  }


  /**
   * Method to set the auto adjust active state
   * @param isActive boolean to set the auto adjust active state
   * @return Command to set the auto adjust active state
   */
  public Command setAutoAdjustActive(boolean isActive){
    return Commands.runOnce(()-> isAutoAdjustActive = isActive);
  }

  
  /**
   * Method to get the auto adjust active state
   * @return boolean of the auto adjust active state
   */
  @AutoLogOutput (key = "Commands/AlignToReef/AutoAdjustActive")
  public boolean getIsAutoAdjustActive(){
    return isAutoAdjustActive;
  }



  @AutoLogOutput (key = "Commands/AlignToReef/TargetPoseXError")
  public double getTargetPoseXError(){
    return m_swerveSubsystem.getState().Pose.getX() - m_targetPose.getX();
  }

  @AutoLogOutput (key = "Commands/AlignToReef/TargetPoseYError")
  public double getTargetPoseYError(){
    return m_swerveSubsystem.getState().Pose.getY() - m_targetPose.getY();
  }

  @AutoLogOutput (key = "Commands/AlignToReef/TargetRotationError")
  public double getTargetRotationError(){
    return m_swerveSubsystem.getState().Pose.getRotation().getDegrees() - m_targetPose.getRotation().getDegrees();
  }


}
