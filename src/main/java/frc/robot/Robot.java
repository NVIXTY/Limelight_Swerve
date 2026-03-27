// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MatchInfo;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private final boolean kUseLimelight = true;

    private double m_teleopStartTime = Double.NaN;

    private static final double kTransitionSeconds = 10.0;
    private static final double kShiftSeconds = 25.0;
    private static final double kNumShifts = 4;
    private static final double kEndgameSeconds = 30.0;
    private static final double kTeleopTotal = kTransitionSeconds + (kShiftSeconds * kNumShifts) + kEndgameSeconds;

    private static final double kBlinkSeconds = 3.0;

    private static final double[] PHASE_BOUNDARIES = {
        kTransitionSeconds,
        kTransitionSeconds + kShiftSeconds,
        kTransitionSeconds + 2 * kShiftSeconds,
        kTransitionSeconds + 3 * kShiftSeconds,
        kTransitionSeconds + 4 * kShiftSeconds,
        kTeleopTotal
    };

    public static final double TELEOP_TOTAL = kTeleopTotal;
    public static final double ENDGAME_SECONDS = kEndgameSeconds;
    public static final double BLINK_SECONDS = kBlinkSeconds;

    public static double timeUntilNextPhaseStatic(double teleopElapsed) {
        for (double b : PHASE_BOUNDARIES) {
            if (b > teleopElapsed) {
                return b - teleopElapsed;
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    public static boolean isHubActiveStatic(double teleopElapsed, String gameData, Optional<Alliance> allianceOpt) {
        if (allianceOpt.isEmpty()) {
            return false;
        }
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }
        if (gameData == null || gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst;
        char c = gameData.charAt(0);
        if (c == 'R') {
            redInactiveFirst = true;
        } else if (c == 'B') {
            redInactiveFirst = false;
        } else {
            return true;
        }
        boolean shift1Active = (allianceOpt.get() == Alliance.Red) ? !redInactiveFirst : redInactiveFirst;
        if (teleopElapsed < kTransitionSeconds) {
            return true;
        }
        if (teleopElapsed < kTransitionSeconds + kShiftSeconds) {
            return shift1Active;
        }
        if (teleopElapsed < kTransitionSeconds + 2 * kShiftSeconds) {
            return !shift1Active;
        }
        if (teleopElapsed < kTransitionSeconds + 3 * kShiftSeconds) {
            return shift1Active;
        }
        if (teleopElapsed < kTransitionSeconds + 4 * kShiftSeconds) {
            return !shift1Active;
        }
        return true;
    }

    public Robot() {
        Logger.recordMetadata("Beta-Bot", "MyProject");

        try {
            Logger.addDataReceiver(new NT4Publisher());
        } catch (Exception ignored) {}

        Logger.start();

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
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-three");
            if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                m_robotContainer.drivetrain.addVisionMeasurement(
                    llMeasurement.pose, llMeasurement.timestampSeconds);
            }
        }

        double now = Timer.getFPGATimestamp();

        double matchTimeLeft = DriverStation.getMatchTime();
        double matchTimeLeftPublished = Double.isFinite(matchTimeLeft) ? matchTimeLeft : Double.NaN;
        Logger.recordOutput("Match/MatchTime", matchTimeLeftPublished);

        double teleopElapsed;
        if (matchTimeLeft > 0.0) {
            teleopElapsed = kTeleopTotal - matchTimeLeft;
            if (teleopElapsed < 0.0) {
                teleopElapsed = 0.0;
            }
        } else {
            if (Double.isNaN(m_teleopStartTime)) {
                m_teleopStartTime = now;
            }
            teleopElapsed = now - m_teleopStartTime;
        }

        double timeToNextPhase = timeUntilNextPhase(teleopElapsed);

        double timeUntilNextPhasePublished = Double.isFinite(timeToNextPhase)
            ? Math.round(timeToNextPhase * 10.0) / 10.0
            : Double.NaN;
        Logger.recordOutput("Match/TimeUntilNextPhase", timeUntilNextPhasePublished);

        String matchClock;
        if (Double.isFinite(matchTimeLeft) && matchTimeLeft >= 0.0) {
            long sec = (long) Math.max(0.0, Math.ceil(matchTimeLeft));
            matchClock = String.format("%02d:%02d", sec / 60, sec % 60);
        } else {
            matchClock = "--:--";
        }
        Logger.recordOutput("Match/Clock", matchClock);
    }

    private double timeUntilNextPhase(double teleopElapsed) {
        if (!DriverStation.isTeleopEnabled()) {
            return Double.POSITIVE_INFINITY;
        }
        for (double b : PHASE_BOUNDARIES) {
            if (b > teleopElapsed) {
                return b - teleopElapsed;
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    public boolean isHubActive(double teleopElapsed, String gameData, Optional<Alliance> allianceOpt) {
        return isHubActiveStatic(teleopElapsed, gameData, allianceOpt);
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
