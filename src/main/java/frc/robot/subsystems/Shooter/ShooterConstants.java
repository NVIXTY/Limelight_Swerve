// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.LoggedTunableNumber;

public class ShooterConstants {
    public static final int kShooterMotorId1 = 12;
    public static final int kShooterMotorId2 = 14;

    public static final int kSupplyCurrentLimit = 60;

    public static final int kStatorCurrentLimit = 120;

    /**
     * Motor velocity Kalman filter time constant (seconds). Smaller = less lag in velocity
     * feedback (faster-looking recovery), larger = smoother/noisier tradeoff. Tune with
     * kP/kD if you see jitter or sluggish response.
     */
    public static final double kVelocityFilterTimeSec = 0.004;

    // Velocity loop: tuned for snappy recovery (was "high accel" 3x kP vs old base).
    public static final double kP = 0.15;
    public static final double kI = 0.0;
    public static final double kD = 0.004;
    public static final double kS = 0.26444;
    public static final double kA = 0.035;
    public static final double kV = 0.13;

    public static final double kVelocityTolerance = 2;

    public static final double kMinIdleVelocity = 5.0;

    public static final double kPrepSpeed = 5;

    /** Wheel RPS when outtaking (sign = reverse). Tune on robot. */
    public static final double kOutshootVelocity = -10.0;

    /** Softer reverse for intake + slow kicker; tune with kOutshootVelocity. */
    public static final double kOutshootSlowVelocity = -5.0;

    public static final double kShooterShuttleSpeed = 50;

    public static InterpolatingDoubleTreeMap kShooterHubMap = new InterpolatingDoubleTreeMap();

    private static final LoggedTunableNumber kshooter15 = new LoggedTunableNumber("Shooter/Hub/1.5", 45, true);
    private static final LoggedTunableNumber kshooter175 = new LoggedTunableNumber("Shooter/Hub/1.75", 43, true);
    private static final LoggedTunableNumber kshooter20 = new LoggedTunableNumber("Shooter/Hub/2.0", 44.5, true);
    private static final LoggedTunableNumber kshooter225 = new LoggedTunableNumber("Shooter/Hub/2.25", 45, true);
    private static final LoggedTunableNumber kshooter25 = new LoggedTunableNumber("Shooter/Hub/2.5", 46, true);
    private static final LoggedTunableNumber kshooter275 = new LoggedTunableNumber("Shooter/Hub/2.75", 48, true);
    private static final LoggedTunableNumber kshooter30 = new LoggedTunableNumber("Shooter/Hub/3.0", 47.5, true);
    private static final LoggedTunableNumber kshooter325 = new LoggedTunableNumber("Shooter/Hub/3.25", 48, true);
    private static final LoggedTunableNumber kshooter35 = new LoggedTunableNumber("Shooter/Hub/3.5", 49.2, true);
    private static final LoggedTunableNumber kshooter375 = new LoggedTunableNumber("Shooter/Hub/3.75", 51, true);
    private static final LoggedTunableNumber kshooter40 = new LoggedTunableNumber("Shooter/Hub/4.0", 51.5, true);
    private static final LoggedTunableNumber kshooter425 = new LoggedTunableNumber("Shooter/Hub/4.25", 52.2, true);
    private static final LoggedTunableNumber kshooter45 = new LoggedTunableNumber("Shooter/Hub/4.5", 53.5, true);
    private static final LoggedTunableNumber kshooter475 = new LoggedTunableNumber("Shooter/Hub/4.75", 54.7, true);
    private static final LoggedTunableNumber kshooter50 = new LoggedTunableNumber("Shooter/Hub/5.0", 55.9, true);
    private static final LoggedTunableNumber kshooter55 = new LoggedTunableNumber("Shooter/Hub/5.5", 58.85, true);
    private static final LoggedTunableNumber kshooter60 = new LoggedTunableNumber("Shooter/Hub/6.0", 60, true);

    public static InterpolatingDoubleTreeMap kShooterFerryMap = new InterpolatingDoubleTreeMap();
    // Ferry-map tunables (1m increments from 3m to 15m)
    private static final LoggedTunableNumber kshooterFerry30 = new LoggedTunableNumber("Shooter/Ferry/3.0", 30.0, true);
    private static final LoggedTunableNumber kshooterFerry40 = new LoggedTunableNumber("Shooter/Ferry/4.0", 34.0, true);
    private static final LoggedTunableNumber kshooterFerry50 = new LoggedTunableNumber("Shooter/Ferry/5.0", 38.8, true);
    private static final LoggedTunableNumber kshooterFerry60 = new LoggedTunableNumber("Shooter/Ferry/6.0", 46.5, true);
    private static final LoggedTunableNumber kshooterFerry70 = new LoggedTunableNumber("Shooter/Ferry/7.0", 54.1, true);
    private static final LoggedTunableNumber kshooterFerry80 = new LoggedTunableNumber("Shooter/Ferry/8.0", 61.7, true);
    private static final LoggedTunableNumber kshooterFerry90 = new LoggedTunableNumber("Shooter/Ferry/9.0", 69.4, true);
    private static final LoggedTunableNumber kshooterFerry100 = new LoggedTunableNumber("Shooter/Ferry/10.0", 76.0, true);
    private static final LoggedTunableNumber kshooterFerry110 = new LoggedTunableNumber("Shooter/Ferry/11.0", 80.0, true);
    private static final LoggedTunableNumber kshooterFerry120 = new LoggedTunableNumber("Shooter/Ferry/12.0", 84.0, true);
    private static final LoggedTunableNumber kshooterFerry130 = new LoggedTunableNumber("Shooter/Ferry/13.0", 88.0, true);
    private static final LoggedTunableNumber kshooterFerry140 = new LoggedTunableNumber("Shooter/Ferry/14.0", 92.0, true);
    private static final LoggedTunableNumber kshooterFerry150 = new LoggedTunableNumber("Shooter/Ferry/15.0", 96.0, true);

    public static double getShooterHubVelocity(double distance) {
        kShooterHubMap.put(1.5, kshooter15.get());
        kShooterHubMap.put(1.75, kshooter175.get());
        kShooterHubMap.put(2.0, kshooter20.get());
        kShooterHubMap.put(2.25, kshooter225.get());
        kShooterHubMap.put(2.5, kshooter25.get());
        kShooterHubMap.put(2.75, kshooter275.get());
        kShooterHubMap.put(3.0, kshooter30.get());
        kShooterHubMap.put(3.25, kshooter325.get());
        kShooterHubMap.put(3.5, kshooter35.get());
        kShooterHubMap.put(3.75, kshooter375.get());
        kShooterHubMap.put(4.0, kshooter40.get());
        kShooterHubMap.put(4.25, kshooter425.get());
        kShooterHubMap.put(4.5, kshooter45.get());
        kShooterHubMap.put(4.75, kshooter475.get());
        kShooterHubMap.put(5.0, kshooter50.get());
        kShooterHubMap.put(5.5, kshooter55.get());
        kShooterHubMap.put(6.0, kshooter60.get());

        return kShooterHubMap.get(distance);
    }

    public static double getShooterFerryVelocity(double distance) {
        kShooterFerryMap.put(3.0, kshooterFerry30.get());
        kShooterFerryMap.put(4.0, kshooterFerry40.get());
        kShooterFerryMap.put(5.0, kshooterFerry50.get());
        kShooterFerryMap.put(6.0, kshooterFerry60.get());
        kShooterFerryMap.put(7.0, kshooterFerry70.get());
        kShooterFerryMap.put(8.0, kshooterFerry80.get());
        kShooterFerryMap.put(9.0, kshooterFerry90.get());
        kShooterFerryMap.put(10.0, kshooterFerry100.get());
        kShooterFerryMap.put(11.0, kshooterFerry110.get());
        kShooterFerryMap.put(12.0, kshooterFerry120.get());
        kShooterFerryMap.put(13.0, kshooterFerry130.get());
        kShooterFerryMap.put(14.0, kshooterFerry140.get());
        kShooterFerryMap.put(15.0, kshooterFerry150.get());

        return kShooterFerryMap.get(distance);
    }
}
