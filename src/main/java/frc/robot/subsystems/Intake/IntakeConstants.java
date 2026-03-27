// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;


public class IntakeConstants {
    public static final int kPivotMotorId = 13;


    public static final double kP = 60;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kCruiseVelocity = 100;
    public static final double kAcceleration = 100;

    
    public static final double kTolerance = 0.02;

    public static final double kSupplyCurrentLimit = 35;

    public static final double kSensorToMechanismRatio = 75;

    public static final double kRollerSpeed = -.6;

    public static final double kIntakeUpPosition = 0.0;
    public static final double kIntakeAgitateStartPos = 0.12;
    public static final double kIntakeAgitateEndPos = 0.05;
    public static final double kAgitateDecaySeconds = 5.5;
    public static final double kIntakeDownPosition = 0.17; // 0.17 is the max down position
    public static final double kIntakeOuttakePosition = 0.1;


    public static final int kRollerMotorId = 15;
    /** Follower roller; opposed to {@link #kRollerMotorId}. */
    public static final int kRollerMotorFollowerId = 22;
    public static final double kRollerSupplyCurrentLimit = 55;


    public static double kIntakeAgitatePosition = .12;


}

