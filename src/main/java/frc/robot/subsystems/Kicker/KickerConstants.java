// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

public class KickerConstants {
    public static final int kKickerMotorId = 9;

    public static final int kKickerSupplyCurrentLimit = 30;

    public static final double kKickerInSpeed = 0.3;
    public static final double kKickerOutSpeed = -kKickerInSpeed;

    /** Stator amps — above this counts as “hit resistance” (tune on robot). */
    public static final double kKickUntilResistanceStatorThresholdAmps = 25.0;
    /** Ignore current checks for this long after starting so the motor can spin up. */
    public static final double kKickUntilResistanceSpinUpSeconds = 0.12;
    /** Stop if threshold never reached (stuck air-shooting). */
    public static final double kKickUntilResistanceTimeoutSeconds = 4.0;
    /** Consecutive loops above threshold before stopping (debounce noise). */
    public static final int kKickUntilResistanceDebounceLoops = 3;

    /** Rotor rotations to unwind opposite direction after resistance (tune on robot). */
    public static final double kAfterResistanceReverseRotations = 0.35;
    public static final double kReversePositionToleranceRot = 0.04;
    public static final double kReverseTimeoutSeconds = 2.0;

    public static final double kReversePositionkP = 10.0;
    public static final double kReversePositionkI = 0.0;
    public static final double kReversePositionkD = 0.02;

    public static final double kReverseMMCruiseRps = 90.0;
    public static final double kReverseMMAccelRps2 = 220.0;
}
