// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class ShooterConstants {
    public static final int kShooterMotorId1 = 12;
    public static final int kShooterMotorId2 = 14;

    public static final int kSupplyCurrentLimit = 35;

    public static final double kP = 0.18249;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.26444;
    public static final double kA = 0.0076942;
    public static final double kV = 0.12232;

    public static final int kAcceleration = 1000;
    public static final int kJerk = 1000;

    public static final double kVelocityTolerance = 2; // RPS 1-100


    public static final double kPrepSpeed = 10; // RPS 1-100

    public static final double kShooterShuttleSpeed = 50; // RPS 1-100


    public static InterpolatingDoubleTreeMap kShooterHubMap = new InterpolatingDoubleTreeMap();

    private static final LoggedTunableNumber kshooter15 = new LoggedTunableNumber("Shooter/Hub/1.5", 30,true);
    private static final LoggedTunableNumber kshooter20 = new LoggedTunableNumber("Shooter/Hub/2.0", 43,true);
    private static final LoggedTunableNumber kshooter25 = new LoggedTunableNumber("Shooter/Hub/2.5", 48,true);
    private static final LoggedTunableNumber kshooter30 = new LoggedTunableNumber("Shooter/Hub/3.0", 50,true);
    private static final LoggedTunableNumber kshooter35 = new LoggedTunableNumber("Shooter/Hub/3.5", 55,true);
    private static final LoggedTunableNumber kshooter40 = new LoggedTunableNumber("Shooter/Hub/4.0", 57,true);
    private static final LoggedTunableNumber kshooter45 = new LoggedTunableNumber("Shooter/Hub/4.5", 65,true);
    private static final LoggedTunableNumber kshooter50 = new LoggedTunableNumber("Shooter/Hub/5.0", 75,true);
    private static final LoggedTunableNumber kshooter55 = new LoggedTunableNumber("Shooter/Hub/5.5", 80,true);
    private static final LoggedTunableNumber kshooter60 = new LoggedTunableNumber("Shooter/Hub/6.0", 90,true);
    private static final LoggedTunableNumber kshooter65 = new LoggedTunableNumber("Shooter/Hub/6.5", 95,true);
    private static final LoggedTunableNumber kshooter70 = new LoggedTunableNumber("Shooter/Hub/7.0", 95,true);

    
    public static InterpolatingDoubleTreeMap kShooterFerryMap = new InterpolatingDoubleTreeMap();


    private static final LoggedTunableNumber kshooterFerry35 = new LoggedTunableNumber("Shooter/Ferry/3.5", 50,true);
    private static final LoggedTunableNumber kshooterFerry40 = new LoggedTunableNumber("Shooter/Ferry/4.0", 50,true);
    private static final LoggedTunableNumber kshooterFerry45 = new LoggedTunableNumber("Shooter/Ferry/4.5", 50,true);
    private static final LoggedTunableNumber kshooterFerry50 = new LoggedTunableNumber("Shooter/Ferry/5.0", 50,true);
    private static final LoggedTunableNumber kshooterFerry55 = new LoggedTunableNumber("Shooter/Ferry/5.5", 50,true);
    private static final LoggedTunableNumber kshooterFerry60 = new LoggedTunableNumber("Shooter/Ferry/6.0", 50,true);
    private static final LoggedTunableNumber kshooterFerry65 = new LoggedTunableNumber("Shooter/Ferry/6.5", 50,true);
    private static final LoggedTunableNumber kshooterFerry70 = new LoggedTunableNumber("Shooter/Ferry/7.0", 50,true);

    public static double getShooterHubVelocity(double distance) {
        
        kShooterHubMap.put(1.5, kshooter15.get());
        kShooterHubMap.put(2.0, kshooter20.get());
        kShooterHubMap.put(2.5, kshooter25.get());
        kShooterHubMap.put(3.0, kshooter30.get());
        kShooterHubMap.put(3.5, kshooter35.get());
        kShooterHubMap.put(4.0, kshooter40.get());
        kShooterHubMap.put(4.5, kshooter45.get());
        kShooterHubMap.put(5.0, kshooter50.get());
        kShooterHubMap.put(5.5, kshooter55.get());
        kShooterHubMap.put(6.0, kshooter60.get());
        kShooterHubMap.put(6.5, kshooter65.get());
        kShooterHubMap.put(7.0, kshooter70.get());

        return kShooterHubMap.get(distance);

    }

    public static double getShooterFerryVelocity(double distance) {
        
        kShooterFerryMap.put(3.5, kshooterFerry35.get());
        kShooterFerryMap.put(4.0, kshooterFerry40.get());
        kShooterFerryMap.put(4.5, kshooterFerry45.get());
        kShooterFerryMap.put(5.0, kshooterFerry50.get());
        kShooterFerryMap.put(5.5, kshooterFerry55.get());
        kShooterFerryMap.put(6.0, kshooterFerry60.get());
        kShooterFerryMap.put(6.5, kshooterFerry65.get());
        kShooterFerryMap.put(7.0, kshooterFerry70.get());

        return kShooterFerryMap.get(distance);

    }
}

