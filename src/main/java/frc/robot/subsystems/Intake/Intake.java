// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX pivotMotor;
  private TalonFXConfiguration pivotConfig;

  private TalonFX rollerMotor;
  private TalonFXConfiguration rollerConfig;

  private MotionMagicVoltage m_motionRequest;

  private IntakeState currentState = IntakeState.STOP;

  /** Creates a new intake. */
  public Intake() {
    pivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);

    pivotConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                          .withInverted(InvertedValue.CounterClockwise_Positive)
                                          .withNeutralMode(NeutralModeValue.Brake))
                        .withSlot0(new Slot0Configs()
                                    .withKP(IntakeConstants.getIntakePivotkP())
                                    .withKI(IntakeConstants.getIntakePivotkI())
                                    .withKD(IntakeConstants.getIntakePivotkD()))
                        .withMotionMagic(new MotionMagicConfigs()
                                        .withMotionMagicCruiseVelocity(IntakeConstants.kCruiseVelocity)
                                        .withMotionMagicAcceleration(IntakeConstants.kAcceleration))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withSupplyCurrentLimit(IntakeConstants.kSupplyCurrentLimit))
                        .withFeedback(new FeedbackConfigs()
                                      .withSensorToMechanismRatio(IntakeConstants.kSensorToMechanismRatio));
    
    pivotMotor.getConfigurator().apply(pivotConfig);

    m_motionRequest = new MotionMagicVoltage(0).withSlot(0);

    pivotMotor.setPosition(0);


    rollerMotor = new TalonFX(IntakeConstants.kRollerMotorId);

    rollerConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.Clockwise_Positive) //Set motor inversion based on mechanism
                                              .withNeutralMode(NeutralModeValue.Brake))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                              .withSupplyCurrentLimit(IntakeConstants.kRollerSupplyCurrentLimit));

    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  public void setGoal(IntakeState desiredState) {
    currentState = desiredState;
    switch (desiredState) {
      case INTAKE:
        setPivotPosition(IntakeConstants.kIntakeDownPosition);
        rollerMotor.set(IntakeConstants.kSpeed);
        break;
      case OUTTAKE:
        setPivotPosition(IntakeConstants.kIntakeDownPosition);
        rollerMotor.set(-IntakeConstants.kSpeed);
        break;
      case AGITATE:
        setPivotPosition(IntakeConstants.kIntakeAgitatePosition);
        rollerMotor.set(IntakeConstants.kSpeed);
        break;
      case STOP:
        pivotMotor.stopMotor();
        rollerMotor.stopMotor();
        break;
      case STOW:
        setPivotPosition(IntakeConstants.kIntakeUpPosition);
        rollerMotor.stopMotor();
        break;
      
    }
  }

  public void rollerIntake() {
    rollerMotor.set(IntakeConstants.kSpeed);
  }

  public void rollerOuttake() {
    rollerMotor.set(-IntakeConstants.kSpeed);
  }

  public void rollerStop() {
    rollerMotor.stopMotor();
  }



  public void pivotUp() {
    pivotMotor.set(IntakeConstants.kSpeed);
  }

  public void pivotDown() {
    pivotMotor.set(-IntakeConstants.kSpeed);
  }

  public void pivotStop() {
    pivotMotor.stopMotor();
  }



  public void zeroIntake() {
    pivotMotor.setPosition(0);
  }

  public void setPivotPosition(double position) {
    pivotMotor.setControl(m_motionRequest.withPosition(position));
  }

 

  @Override
  public void periodic() {
    logMotorData();
  }

  public boolean isAtSetpoint() {
    return Math.abs((pivotMotor.getPosition().getValueAsDouble()) - (m_motionRequest.Position)) <= IntakeConstants.kTolerance;
  }

  public void logMotorData() {

    Logger.recordOutput("Subsystems/Intake/IntakeState", currentState.name());
    
    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/PivotMotorSpeed", pivotMotor.get());
    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/PivotMotorSupplyCurrent", pivotMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/PivotMotorStatorCurrent", pivotMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/PivotMotorVoltage", pivotMotor.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/PivotMotorPosition", pivotMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/PivotMotorSetpoint", m_motionRequest.Position);
    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/IsAtSetpoint", isAtSetpoint());

    Logger.recordOutput("Subsystems/Intake/Basic/Roller/RollerMotorSpeed", rollerMotor.get());
    Logger.recordOutput("Subsystems/Intake/Basic/Roller/RollerMotorSupplyCurrent", rollerMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Roller/RollerMotorStatorCurrent", rollerMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Roller/RollerMotorVoltage", rollerMotor.getMotorVoltage().getValueAsDouble());
}
}
