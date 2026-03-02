// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class HoodConstants {
    public static final int kHoodMotorId = 22;


    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", 1,true);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI", 0,true);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", 0,true);

    public static final double kCruiseVelocity = 500;
    public static final double kAcceleration = 800;

    
    public static final double kTolerance = 0.05;

    public static final double kSupplyCurrentLimit = 35;

    public static final double kSensorToMechanismRatio = 30;

    public static final double kSpeed = 0.1;

    public static final double kTrenchPosition = 0.0;
    public static final double kHubPosition = 0.3;
    public static final double kFerryPosition = 0.6;


    public static double getHoodkP(){
        return kP.get();
    }

    public static double getHoodkI(){
        return kI.get();
    }

    public static double getHoodkD(){
        return kD.get();
    }

}


