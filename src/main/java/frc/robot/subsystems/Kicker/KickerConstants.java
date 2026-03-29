// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

public class KickerConstants {
    public static final int kKickerMotorId = 9;

    public static final int kKickerSupplyCurrentLimit = 30;

    public static final double kKickerInSpeed = 0.25;
    public static final double kKickerOutSpeed = -kKickerInSpeed;

    /** Slow-kick multiplier: slow kick = kKickerInSpeed * multiplier. Tune on robot. */
    public static final double kKickerSlowKickMultiplier = 0.2;

    /** Rotor rotations to unwind after releasing slow kick (same direction as old resistance reverse). */
    public static final double kSlowKickReverseRotations = 0.09;
    public static final double kReversePositionToleranceRot = 0.04;
    public static final double kReverseTimeoutSeconds = 2.0;

    public static final double kReversePositionkP = 10.0;
    public static final double kReversePositionkI = 0.0;
    public static final double kReversePositionkD = 0.02;

    public static final double kReverseMMCruiseRps = 10.0;
    public static final double kReverseMMAccelRps2 = 220.0;
}
