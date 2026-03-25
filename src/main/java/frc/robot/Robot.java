// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MatchInfo;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private final boolean kUseLimelight = true;

    // Match phase / hub active dashboard
    private double m_teleopStartTime = Double.NaN;
    private static final double kTransitionSeconds = 10.0;
    private static final double kShiftSeconds = 25.0;
    private static final double kNumShifts = 4;
    private static final double kEndgameSeconds = 30.0; // last endgame length
    private static final double kTeleopTotal = kTransitionSeconds + (kShiftSeconds * kNumShifts) + kEndgameSeconds; // 140s
    private static final double kBlinkSeconds = 3.0; // seconds to blink before phase change
    // Precompute phase boundaries once to avoid per-loop allocations
    private final double[] m_phaseBoundaries = new double[] {
        kTransitionSeconds,
        kTransitionSeconds + kShiftSeconds,
        kTransitionSeconds + 2 * kShiftSeconds,
        kTransitionSeconds + 3 * kShiftSeconds,
        kTransitionSeconds + 4 * kShiftSeconds,
        kTeleopTotal
    };

    // Expose a few static constants and helpers for external sendables/helpers
    public static final double TELEOP_TOTAL = kTeleopTotal;
    public static final double ENDGAME_SECONDS = kEndgameSeconds;
    public static final double BLINK_SECONDS = kBlinkSeconds;

    public static double timeUntilNextPhaseStatic(double teleopElapsed) {
        double[] boundaries = new double[] {
            kTransitionSeconds,
            kTransitionSeconds + kShiftSeconds,
            kTransitionSeconds + 2 * kShiftSeconds,
            kTransitionSeconds + 3 * kShiftSeconds,
            kTransitionSeconds + 4 * kShiftSeconds,
            kTeleopTotal
        };
        for (double b : boundaries) if (b > teleopElapsed) return b - teleopElapsed;
        return Double.POSITIVE_INFINITY;
    }

    

    public static boolean isHubActiveStatic(double teleopElapsed, String gameData, Optional<Alliance> allianceOpt) {
        if (allianceOpt.isEmpty()) return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;
        if (gameData == null || gameData.isEmpty()) return true;
        boolean redInactiveFirst;
        char c = gameData.charAt(0);
        if (c == 'R') redInactiveFirst = true;
        else if (c == 'B') redInactiveFirst = false;
        else return true;
        boolean shift1Active = (allianceOpt.get() == Alliance.Red) ? !redInactiveFirst : redInactiveFirst;
        if (teleopElapsed < kTransitionSeconds) return true;
        if (teleopElapsed < kTransitionSeconds + kShiftSeconds) return shift1Active;
        if (teleopElapsed < kTransitionSeconds + 2 * kShiftSeconds) return !shift1Active;
        if (teleopElapsed < kTransitionSeconds + 3 * kShiftSeconds) return shift1Active;
        if (teleopElapsed < kTransitionSeconds + 4 * kShiftSeconds) return !shift1Active;
        return true;
    }



    public Robot() {

        Logger.recordMetadata("Beta-Bot", "MyProject"); // Set a metadata value

    // Enable NT4Publisher so Junction Logger outputs (Logger.recordOutput) are
    // published to NetworkTables and visible to AdvantageKit. WPILOGWriter
    // (file logging) remains disabled to avoid file I/O overhead.
    // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    try {
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } catch (Exception ex) {
        // If NT4Publisher fails to initialize, continue without it.
    }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    
    // Default the dashboard toggle once at startup so the driver can flip it
    // at any time during the match. Do NOT overwrite this value in
    // robotPeriodic; reading and writing should be operator-driven.
    // Register a single Sendable to expose match/timer/hub state
    SmartDashboard.putData("Match Info", new MatchInfo());

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        
         if (kUseLimelight) {
            var driveState = m_robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            LimelightHelpers.SetRobotOrientation("limelight-three", headingDeg, 0, 0, 0, 0, 0);

            // Use MegaTag2 (orb) pose only. This keeps behavior stable and avoids
            // performing gyro/pose resets from MT1 logic.
            LimelightHelpers.SetRobotOrientation("limelight-three", headingDeg, 0, 0, 0, 0, 0);
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-three");
            if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
            }
        }

        // --- Match phase dashboard / Hub active boolean + timers ---
        double now = Timer.getFPGATimestamp();

        // gameData and hub override handled by the single Sendable; avoid per-loop dashboard writes here.

        // Match time and teleop elapsed (compute once per loop)
    double matchTimeLeft = DriverStation.getMatchTime();
    double matchTimeLeftPublished = Double.isFinite(matchTimeLeft) ? matchTimeLeft : Double.NaN;
    Logger.recordOutput("Match/MatchTime", matchTimeLeftPublished);

        double teleopElapsed;
        if (matchTimeLeft > 0.0) {
            teleopElapsed = kTeleopTotal - matchTimeLeft;
            if (teleopElapsed < 0.0) teleopElapsed = 0.0;
        } else {
            if (Double.isNaN(m_teleopStartTime)) {
                m_teleopStartTime = now;
            }
            teleopElapsed = now - m_teleopStartTime;
        }

        // Determine hub active using the same teleopElapsed calculation to avoid
        // duplicate DriverStation queries and to make the decision consistent.
        // HubActive and blink behavior provided by the Match Info Sendable.
        double timeToNextPhase = timeUntilNextPhase(teleopElapsed);

        // Time until next phase (low-chatter quantized value)
    double timeUntilNextPhasePublished = Double.isFinite(timeToNextPhase)
        ? Math.round(timeToNextPhase * 10.0) / 10.0
        : Double.NaN;
    Logger.recordOutput("Match/TimeUntilNextPhase", timeUntilNextPhasePublished);

        // Time-until-endgame removed (no longer published per user request)
        // Human-readable match clock (MM:SS) for the dashboard. If unknown, show "--:--".
        String matchClock;
        if (Double.isFinite(matchTimeLeft) && matchTimeLeft >= 0.0) {
            long sec = (long) Math.max(0.0, Math.ceil(matchTimeLeft));
            matchClock = String.format("%02d:%02d", sec / 60, sec % 60);
        } else {
            matchClock = "--:--";
        }
    Logger.recordOutput("Match/Clock", matchClock);
    }



    /**
     * Returns time in seconds until the next teleop phase boundary. If not in teleop
     * or no next phase, returns Double.POSITIVE_INFINITY.
     */
    private double timeUntilNextPhase(double teleopElapsed) {
        // Only meaningful during teleop
        if (!DriverStation.isTeleopEnabled()) return Double.POSITIVE_INFINITY;

        for (double b : m_phaseBoundaries) {
            if (b > teleopElapsed) return b - teleopElapsed;
        }
        return Double.POSITIVE_INFINITY;
    }

    /**
     * Determine whether the hub is active based on DriverStation state and FMS
     * game-specific message. Returns true when the hub should be considered active.
     */
public boolean isHubActive(double teleopElapsed, String gameData, Optional<Alliance> allianceOpt) {
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (allianceOpt.isEmpty()) return false;
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) return true;
        // If we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) return false;

        // If we have no game data, assume early teleop and hub active.
        if (gameData == null || gameData.isEmpty()) return true;

        boolean redInactiveFirst;
        char c = gameData.charAt(0);
        if (c == 'R') redInactiveFirst = true;
        else if (c == 'B') redInactiveFirst = false;
        else return true; // invalid data -> assume active

        boolean shift1Active = (allianceOpt.get() == Alliance.Red) ? !redInactiveFirst : redInactiveFirst;

        // Map elapsed into phases relative to kTransitionSeconds and kShiftSeconds
        if (teleopElapsed < kTransitionSeconds) {
                return true; // transition
        } else if (teleopElapsed < kTransitionSeconds + kShiftSeconds) {
                return shift1Active; // shift 1
        } else if (teleopElapsed < kTransitionSeconds + 2 * kShiftSeconds) {
                return !shift1Active; // shift 2
        } else if (teleopElapsed < kTransitionSeconds + 3 * kShiftSeconds) {
                return shift1Active; // shift 3
        } else if (teleopElapsed < kTransitionSeconds + 4 * kShiftSeconds) {
                return !shift1Active; // shift 4
        }
        // Endgame or after: hub active
        return true;
}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            try {
                CommandScheduler.getInstance().schedule(m_autonomousCommand);
            } catch (Exception ex) {
                // Protect the robot from crashing due to composition/scheduling errors.
                System.err.println("Failed to schedule autonomous command: " + ex);
                ex.printStackTrace();
                Logger.recordOutput("Autonomous/ScheduleError", ex.toString());
            }
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
