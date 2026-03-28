// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

public enum ShooterState {
    HUB,
    LEFT_FERRY,
    RIGHT_FERRY,
    IDLE,
    /** Reverse shooter while outtaking (see RobotContainer outtake). */
    OUTSHOOT,
    /** Lower reverse speed paired with kicker slow kick during intake. */
    OUTSHOOT_SLOW,
    STOP
}
