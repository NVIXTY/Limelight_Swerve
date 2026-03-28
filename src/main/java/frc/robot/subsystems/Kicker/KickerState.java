// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

public enum KickerState {
    KICK,
    OUTKICK,
    /** Intake assist: low duty cycle forward; reverse unwind on exit (see KickerConstants). */
    SLOW_KICK,
    STOP
}
