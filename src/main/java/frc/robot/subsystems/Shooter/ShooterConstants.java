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

    // PID/Feedforward tuning — adjusted to reduce overshoot while keeping
    // reasonably fast spin-up. These are conservative values; tune further
    // on the robot with telemetry.
    public static final double kP = 0.05; 
    public static final double kI = 0.0;
    public static final double kD = 0.002; 
    public static final double kS = 0.26444;
    public static final double kA = 0.01;
    public static final double kV = 0.13;

    public static final int kAcceleration = 2000;
    public static final int kJerk = 2000;

    public static final double kVelocityTolerance = 2; // RPS 1-100

    public static final double kMinIdleVelocity = 5.0;

    public static final double kPrepSpeed = 5; // RPS 1-100

    public static final double kShooterShuttleSpeed = 50; // RPS 1-100


    public static InterpolatingDoubleTreeMap kShooterHubMap = new InterpolatingDoubleTreeMap();

    private static final LoggedTunableNumber kshooter15 = new LoggedTunableNumber("Shooter/Hub/1.5", 45, true); // TUNED
    private static final LoggedTunableNumber kshooter175 = new LoggedTunableNumber("Shooter/Hub/1.75", 45, true); // TUNED
    private static final LoggedTunableNumber kshooter20 = new LoggedTunableNumber("Shooter/Hub/2.0", 47, true); // TUNED
    private static final LoggedTunableNumber kshooter225 = new LoggedTunableNumber("Shooter/Hub/2.25", 47, true); // TUNED 
    private static final LoggedTunableNumber kshooter25 = new LoggedTunableNumber("Shooter/Hub/2.5", 46, true); // TUNED
    private static final LoggedTunableNumber kshooter275 = new LoggedTunableNumber("Shooter/Hub/2.75", 49, true); // TUNED
    private static final LoggedTunableNumber kshooter30 = new LoggedTunableNumber("Shooter/Hub/3.0", 50, true); // TUNED
    private static final LoggedTunableNumber kshooter325 = new LoggedTunableNumber("Shooter/Hub/3.25", 51, true); // TUNED
    private static final LoggedTunableNumber kshooter35 = new LoggedTunableNumber("Shooter/Hub/3.5", 50, true); // TUNED
    private static final LoggedTunableNumber kshooter375 = new LoggedTunableNumber("Shooter/Hub/3.75", 53, true); // TUNED
    private static final LoggedTunableNumber kshooter40 = new LoggedTunableNumber("Shooter/Hub/4.0", 53, true); // TUNED
    private static final LoggedTunableNumber kshooter425 = new LoggedTunableNumber("Shooter/Hub/4.25", 54.5, true); // TUNED
    private static final LoggedTunableNumber kshooter45 = new LoggedTunableNumber("Shooter/Hub/4.5", 55.5, true); // TUNED
    private static final LoggedTunableNumber kshooter475 = new LoggedTunableNumber("Shooter/Hub/4.75",  56.5, true); // TUNED
    private static final LoggedTunableNumber kshooter50 = new LoggedTunableNumber("Shooter/Hub/5.0", 56.5, true); // TUNED
    private static final LoggedTunableNumber kshooter55 = new LoggedTunableNumber("Shooter/Hub/5.5", 60.5, true); // TUNED
    private static final LoggedTunableNumber kshooter60 = new LoggedTunableNumber("Shooter/Hub/6.0", 63.5, true); // TUNED

    public static InterpolatingDoubleTreeMap kShooterFerryMap = new InterpolatingDoubleTreeMap();

    // Ferry map: from 4.5 m up to 10.0 m in 0.5 m increments. Default values
    // linearly interpolate from 55 at 4.5 m to 97 at 10.0 m.
    private static final LoggedTunableNumber kshooterFerry45 = new LoggedTunableNumber("Shooter/Ferry/4.5", 35.0, true);
    private static final LoggedTunableNumber kshooterFerry50 = new LoggedTunableNumber("Shooter/Ferry/5.0", 38.8, true);
    private static final LoggedTunableNumber kshooterFerry55 = new LoggedTunableNumber("Shooter/Ferry/5.5", 42.6, true);
    private static final LoggedTunableNumber kshooterFerry60 = new LoggedTunableNumber("Shooter/Ferry/6.0", 46.5, true);
    private static final LoggedTunableNumber kshooterFerry65 = new LoggedTunableNumber("Shooter/Ferry/6.5", 50.3, true);
    private static final LoggedTunableNumber kshooterFerry70 = new LoggedTunableNumber("Shooter/Ferry/7.0", 54.1, true);
    private static final LoggedTunableNumber kshooterFerry75 = new LoggedTunableNumber("Shooter/Ferry/7.5", 57.9, true);
    private static final LoggedTunableNumber kshooterFerry80 = new LoggedTunableNumber("Shooter/Ferry/8.0", 61.7, true);
    private static final LoggedTunableNumber kshooterFerry85 = new LoggedTunableNumber("Shooter/Ferry/8.5", 65.5, true);
    private static final LoggedTunableNumber kshooterFerry90 = new LoggedTunableNumber("Shooter/Ferry/9.0", 69.4, true);
    private static final LoggedTunableNumber kshooterFerry95 = new LoggedTunableNumber("Shooter/Ferry/9.5", 73.2, true);
    private static final LoggedTunableNumber kshooterFerry100 = new LoggedTunableNumber("Shooter/Ferry/10.0", 87.0, true);

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
        
    kShooterFerryMap.put(4.5, kshooterFerry45.get());
    kShooterFerryMap.put(5.0, kshooterFerry50.get());
    kShooterFerryMap.put(5.5, kshooterFerry55.get());
    kShooterFerryMap.put(6.0, kshooterFerry60.get());
    kShooterFerryMap.put(6.5, kshooterFerry65.get());
    kShooterFerryMap.put(7.0, kshooterFerry70.get());
    kShooterFerryMap.put(7.5, kshooterFerry75.get());
    kShooterFerryMap.put(8.0, kshooterFerry80.get());
    kShooterFerryMap.put(8.5, kshooterFerry85.get());
    kShooterFerryMap.put(9.0, kshooterFerry90.get());
    kShooterFerryMap.put(9.5, kshooterFerry95.get());
    kShooterFerryMap.put(10.0, kshooterFerry100.get());

        return kShooterFerryMap.get(distance);

    }
}

