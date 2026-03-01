// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {

  private TalonFX KickerMotor;
  private TalonFXConfiguration KickerConfig;

  private KickerState currentState = KickerState.STOP;

  /** Creates a new Kicker. */
  public Kicker() {

  KickerMotor = new TalonFX(KickerConstants.kKickerMotorId);

  KickerConfig = new TalonFXConfiguration()
                      .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.Clockwise_Positive) //Set motor inversion based on mechanism
                                            .withNeutralMode(NeutralModeValue.Brake))
                      .withCurrentLimits(new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimit(KickerConstants.kKickerSupplyCurrentLimit));

  KickerMotor.getConfigurator().apply(KickerConfig);

  }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotorData();

  }

  public void setGoal(KickerState desiredState) {
    currentState = desiredState;
    switch (desiredState) {
      case KICK:
        KickerMotor.set(KickerConstants.kKickerInSpeed);
        break;
      case OUTKICK:
        KickerMotor.set(KickerConstants.kKickerOutSpeed);
        break;
      case STOP:
        KickerMotor.stopMotor();
        break;
    }
  }

  private void logMotorData(){
    Logger.recordOutput("Subsystems/Kicker/KickerState", currentState.name());
    Logger.recordOutput("Subsystems/Kicker/Basic/KickerMotorSpeed", KickerMotor.get());
    Logger.recordOutput("Subsystems/Kicker/Basic/KickerMotorSupplyCurrent", KickerMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Kicker/Basic/KickerMotorStatorCurrent", KickerMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Kicker/Basic/KickerMotorVoltage", KickerMotor.getMotorVoltage().getValueAsDouble());
  }
}
