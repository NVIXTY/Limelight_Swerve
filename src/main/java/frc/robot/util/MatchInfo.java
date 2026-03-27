package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Thin Sendable so "Match Info" shows up on SD; notifier pushes time elsewhere. */
public class MatchInfo implements Sendable {
    private static final double RECORD_INTERVAL_SECONDS = 0.1;

    private static final Notifier s_notifier = new Notifier(() -> {
        SmartDashboard.putNumber("Match/Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Match/ShiftTimer", MatchInfo.getShiftTimer());
    });

    static {
        s_notifier.startPeriodic(RECORD_INTERVAL_SECONDS);
    }

    public static double getShiftTimer() {
        double matchTime = DriverStation.getMatchTime();

        if (matchTime >= 130) {
            return matchTime - 130;
        } else if (matchTime >= 105) {
            return matchTime - 105;
        } else if (matchTime >= 80) {
            return matchTime - 80;
        } else if (matchTime >= 55) {
            return matchTime - 55;
        } else if (matchTime >= 30) {
            return matchTime - 30;
        } else {
            return -1;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("MatchInfo");
    }
}
