package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * MatchInfo sendable with a very small-footprint match timer for AdvantageKit/Elastic.
 *
 * This class intentionally exposes no dashboard properties (keeps the
 * "Match Info" entry minimal) but does publish the current match time
 * to the Junction logger at a low rate so external telemetry (AdvantageKit,
 * Elastic, etc.) can ingest it with minimal storage use on the RoboRIO.
 */
public class MatchInfo implements Sendable {
    // Record match time once per second (seconds remaining as a double).
    // Using the Junction Logger is compact; AdvantageKit/Elastic can subscribe
    // to this recorded key without needing additional NetworkTables state.
    private static final double RECORD_INTERVAL_SECONDS = 0.1;

  private static final Notifier s_notifier = new Notifier(() -> {
    // Publish match time to NetworkTables so AdvantageKit/Elastic can ingest it
    // directly from SmartDashboard/NT with minimal footprint on the RoboRIO.
    SmartDashboard.putNumber("Match/Time", DriverStation.getMatchTime());
    // Publish the computed shift timer so external tools can find it easily.
    SmartDashboard.putNumber("Match/ShiftTimer", MatchInfo.getShiftTimer());
  });

    static {
        // Start periodic recording. Static init ensures this runs once when the
        // class is loaded without requiring further wiring elsewhere.
        s_notifier.startPeriodic(RECORD_INTERVAL_SECONDS);
    }

  public static double getShiftTimer() {
    double matchTime = DriverStation.getMatchTime();

    if (matchTime >= 130) {
      // Transition shift, hub is active.
      return matchTime - 130;
    } else if (matchTime >= 105) {
      // Shift 1
      return matchTime - 105; 
    } else if (matchTime >= 80) {
      // Shift 2
      return matchTime - 80;
    } else if (matchTime >= 55) {
      // Shift 3
      return matchTime - 55;
    } else if (matchTime >= 30) {
      // Shift 4
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

