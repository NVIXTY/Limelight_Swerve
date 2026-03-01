// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private TalonFX ClimberMotor;
  private TalonFXConfiguration ClimberConfig;

  private ClimberState currentState = ClimberState.STOP;

  /** Creates a new Climber. */
  public Climber() {

  ClimberMotor = new TalonFX(ClimberConstants.kClimberMotorId);

  ClimberConfig = new TalonFXConfiguration()
                      .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.Clockwise_Positive) //Set motor inversion based on mechanism
                                            .withNeutralMode(NeutralModeValue.Brake))
                      .withCurrentLimits(new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimit(ClimberConstants.kClimberSupplyCurrentLimit))
                      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                                            .withForwardSoftLimitEnable(true)
                                            .withReverseSoftLimitEnable(true)
                                            .withForwardSoftLimitThreshold(ClimberConstants.kClimberForwardSoftLimitThreshold)
                                            .withReverseSoftLimitThreshold(ClimberConstants.kClimberReverseSoftLimitThreshold));

  ClimberMotor.getConfigurator().apply(ClimberConfig);

  }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotorData();

  }

  public void setGoal(ClimberState desiredState) {
    currentState = desiredState;
    switch (desiredState) {
      case EXTEND:
        ClimberMotor.set(ClimberConstants.kClimberInSpeed);
        break;
      case RETRACT:
        ClimberMotor.set(ClimberConstants.kClimberOutSpeed);
        break;
      case STOP:
        ClimberMotor.stopMotor();
        break;
    }
  }

  private void logMotorData(){
    Logger.recordOutput("Subsystems/Climber/ClimberState", currentState.name());
    Logger.recordOutput("Subsystems/Climber/Basic/ClimberMotorSpeed", ClimberMotor.get());
    Logger.recordOutput("Subsystems/Climber/Basic/ClimberMotorSupplyCurrent", ClimberMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Climber/Basic/ClimberMotorStatorCurrent", ClimberMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Climber/Basic/ClimberMotorVoltage", ClimberMotor.getMotorVoltage().getValueAsDouble());
  }
}
