// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static class PoseConstants {
    public static final Pose2d BLUE_HUB = new Pose2d(4.625,4.035, Rotation2d.kZero);
    public static final Pose2d RED_HUB = FlippingUtil.flipFieldPose(BLUE_HUB);

    public static final Pose2d BLUE_OUTPOST_SHUTTLING = new Pose2d(2.500, 2.000, Rotation2d.kZero);
    public static final Pose2d BLUE_DEPOT_SHUTTLING = new Pose2d(2.500, 6.000, Rotation2d.kZero);

    public static final Pose2d RED_DEPOT_SHUTTLING = FlippingUtil.flipFieldPose(BLUE_DEPOT_SHUTTLING);
    public static final Pose2d RED_OUTPOST_SHUTTLING = FlippingUtil.flipFieldPose(BLUE_OUTPOST_SHUTTLING);

  }







}