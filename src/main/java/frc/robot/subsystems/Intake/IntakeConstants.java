// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class IntakeConstants {
    public static final int kPivotMotorId = 22;


    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Tuning/Intake/Pivot/kP", 1,true);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Tuning/Intake/Pivot/kI", 0,true);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Tuning/Intake/Pivot/kD", 0,true);

    public static final double kCruiseVelocity = 500;
    public static final double kAcceleration = 800;

    
    public static final double kTolerance = 0.05;

    public static final double kSupplyCurrentLimit = 35;

    public static final double kSensorToMechanismRatio = 75;

    public static final double kSpeed = 0.75;

    public static final double kIntakeUpPosition = 0.0;
    public static final double kIntakeAgitatePosition = 0.3;
    public static final double kIntakeDownPosition = 0.6;


    public static final int kRollerMotorId = 23;
    public static final double kRollerSupplyCurrentLimit = 35;



    public static double getIntakePivotkP(){
        return kP.get();
    }

    public static double getIntakePivotkI(){
        return kI.get();
    }

    public static double getIntakePivotkD(){
        return kD.get();
    }

}


