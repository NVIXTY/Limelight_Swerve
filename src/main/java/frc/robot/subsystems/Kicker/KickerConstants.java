// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class KickerConstants {
    public static final int kKickerMotorId = 9;

    public static final int kKickerSupplyCurrentLimit = 70;

    public static final double kKickerInSpeed = 0.225;
    public static final double kKickerOutSpeed = -kKickerInSpeed;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Tuning/Kicker/Pivot/kP", 0,true);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Tuning/Kicker/Pivot/kI", 10,true);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Tuning/Kicker/Pivot/kD", 0,true);

    public static double getKickerPivotkP(){
        return kP.get();
    }

    public static double getKickerPivotkI(){
        return kI.get();
    }

    public static double getKickerPivotkD(){
        return kD.get();
    }

}