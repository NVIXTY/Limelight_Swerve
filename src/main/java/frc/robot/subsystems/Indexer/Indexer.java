// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private TalonFX indexerMotor;
  private TalonFXConfiguration indexerConfig;

  private IndexerState currentState = IndexerState.STOP;

  /** Creates a new Indexer. */
  public Indexer() {

  indexerMotor = new TalonFX(IndexerConstants.kIndexerMotorId);

  indexerConfig = new TalonFXConfiguration()
                      .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.Clockwise_Positive) //Set motor inversion based on mechanism
                                            .withNeutralMode(NeutralModeValue.Brake))
                      .withCurrentLimits(new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimit(IndexerConstants.kIndexerSupplyCurrentLimit));

  indexerMotor.getConfigurator().apply(indexerConfig);

  }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotorData();

  }

  public void setGoal(IndexerState desiredState) {
    currentState = desiredState;
    switch (desiredState) {
      case INDEX:
        indexerMotor.set(IndexerConstants.kIndexerInSpeed);
        break;
      case OUTDEX:
        indexerMotor.set(IndexerConstants.kIndexerOutSpeed);
        break;
      case STOP:
        indexerMotor.stopMotor();
        break;
    }
  }

  private void logMotorData(){
    Logger.recordOutput("Subsystems/Indexer/IndexerState", currentState.name());
    Logger.recordOutput("Subsystems/Indexer/Basic/indexerMotorSpeed", indexerMotor.get());
    Logger.recordOutput("Subsystems/Indexer/Basic/indexerMotorSupplyCurrent", indexerMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Indexer/Basic/indexerMotorStatorCurrent", indexerMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Indexer/Basic/indexerMotorVoltage", indexerMotor.getMotorVoltage().getValueAsDouble());
  }
}
