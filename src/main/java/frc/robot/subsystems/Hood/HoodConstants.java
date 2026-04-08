// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.LoggedTunableNumber;

public class HoodConstants {
    public static final int kHoodMotorId = 10;

    public static final double kP = 500;
    public static final double kI = 0;
    public static final double kD = 4;

    public static final double kCruiseVelocity = 80 * 30; // converted to motor-rotations units
    public static final double kAcceleration = 70 * 30; // converted to motor-rotations units

    public static final double kTolerance = 0.000001 * 30;

    public static final double kSupplyCurrentLimit = 35;

    public static final double kSensorToMechanismRatio = 30.0;

    public static final double kSpeed = 0.05;

    public static final double kTrenchPosition = 0.0;
    public static final double kHubPosition = 0.065; 
    public static final double kFerryPosition = 0.1; 

    public static InterpolatingDoubleTreeMap kHoodMap = new InterpolatingDoubleTreeMap();

    private static final LoggedTunableNumber kHood1 = new LoggedTunableNumber("Hood/Field/1.0", 0.0, true);
    private static final LoggedTunableNumber kHood2 = new LoggedTunableNumber("Hood/Field/2.0", 0.025, true);
    private static final LoggedTunableNumber kHood3 = new LoggedTunableNumber("Hood/Field/3.0", 0.035, true);
    private static final LoggedTunableNumber kHood4 = new LoggedTunableNumber("Hood/Field/4.0", 0.045, true);
    private static final LoggedTunableNumber kHood5 = new LoggedTunableNumber("Hood/Field/5.0", 0.055, true);
    private static final LoggedTunableNumber kHood6 = new LoggedTunableNumber("Hood/Field/6.0", kHubPosition, true);

    public static double getHoodPosition(double distance) {
        // Ensure a 0-meter entry exists so sotm works well moving forwards
        kHoodMap.put(0.0, 0.0);
        kHoodMap.put(1.0, kHood1.get());
        kHoodMap.put(2.0, kHood2.get());
        kHoodMap.put(3.0, kHood3.get());
        kHoodMap.put(4.0, kHood4.get());
        kHoodMap.put(5.0, kHood5.get());
        kHoodMap.put(6.0, kHood6.get());

        return kHoodMap.get(distance);
    }
}
