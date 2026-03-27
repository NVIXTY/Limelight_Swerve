// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.LoggedTunableNumber;

public class HoodConstants {
    public static final int kHoodMotorId = 10;

    public static final double kP = 255;
    public static final double kI = 0;
    public static final double kD = 4;

    public static final double kCruiseVelocity = 80;
    public static final double kAcceleration = 70;

    public static final double kTolerance = 0.000001;

    public static final double kSupplyCurrentLimit = 35;

    public static final double kSensorToMechanismRatio = 30;

    public static final double kSpeed = 0.05;

    public static final double kTrenchPosition = 0.0;
    public static final double kHubPosition = 0.07;
    public static final double kFerryPosition = 0.122;

    public static InterpolatingDoubleTreeMap kHoodMap = new InterpolatingDoubleTreeMap();

    private static final LoggedTunableNumber kHood0 = new LoggedTunableNumber("Hood/Field/0.0", kTrenchPosition, true);
    private static final LoggedTunableNumber kHood125 = new LoggedTunableNumber("Hood/Field/1.25", 0.02, true);
    private static final LoggedTunableNumber kHood25 = new LoggedTunableNumber("Hood/Field/2.5", 0.04, true);
    private static final LoggedTunableNumber kHood375 = new LoggedTunableNumber("Hood/Field/3.75", 0.06, true);
    private static final LoggedTunableNumber kHood5 = new LoggedTunableNumber("Hood/Field/5.0", 0.065, true);
    private static final LoggedTunableNumber kHood6 = new LoggedTunableNumber("Hood/Field/6.0", kHubPosition, true);

    public static double getHoodPosition(double distance) {
        kHoodMap.put(0.0, kHood0.get());
        kHoodMap.put(1.25, kHood125.get());
        kHoodMap.put(2.5, kHood25.get());
        kHoodMap.put(3.75, kHood375.get());
        kHoodMap.put(5.0, kHood5.get());
        kHoodMap.put(6.0, kHood6.get());

        return kHoodMap.get(distance);
    }
}
