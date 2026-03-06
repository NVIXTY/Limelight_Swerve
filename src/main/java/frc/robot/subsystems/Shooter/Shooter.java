// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.DriveConstants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterMotor1;
  private TalonFX shooterMotor2;
  private TalonFXConfiguration shooterConfig;

  private MotionMagicVelocityVoltage m_motionRequest;

  private ShooterState currentState = ShooterState.STOP;

  private CommandSwerveDrivetrain m_swerveSubsystem;

  private double m_hubDistance = 0.0;
  private double m_LeftFerryDistance = 0.0;
  private double m_RightFerryDistance = 0.0;

  public Shooter(CommandSwerveDrivetrain swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;
    shooterMotor1 = new TalonFX(ShooterConstants.kShooterMotorId1);
    shooterMotor2 = new TalonFX(ShooterConstants.kShooterMotorId2);

    shooterConfig = new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs()
                                    .withInverted(InvertedValue.CounterClockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Coast))
                    .withSlot0(new Slot0Configs()
                              .withKP(ShooterConstants.kP)
                              .withKI(ShooterConstants.kI)
                              .withKD(ShooterConstants.kD)
                              .withKS(ShooterConstants.kS)
                              .withKA(ShooterConstants.kA)
                              .withKV(ShooterConstants.kV))
                    .withMotionMagic(new MotionMagicConfigs()
                                    .withMotionMagicAcceleration(ShooterConstants.kAcceleration)
                                    .withMotionMagicJerk(ShooterConstants.kJerk))
                    .withCurrentLimits(new CurrentLimitsConfigs()
                                    .withSupplyCurrentLimit(ShooterConstants.kSupplyCurrentLimit));
    shooterMotor1.getConfigurator().apply(shooterConfig);

    shooterMotor2.setControl(new Follower(ShooterConstants.kShooterMotorId1, MotorAlignmentValue.Opposed));
    shooterMotor2.getConfigurator().apply(shooterConfig);

    m_motionRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

  }
  @Override
  public void periodic() {
    logMotorData();
  }

  public void setGoal(ShooterState desiredState) {
    currentState = desiredState;
    Translation2d currentTranslation2d = m_swerveSubsystem.getState().Pose.getTranslation();
    switch (desiredState) {
      case HUB:
        m_hubDistance = currentTranslation2d.getDistance(DriveConstants.getHubPose().getTranslation());
        setShooterVelocity(ShooterConstants.getShooterHubVelocity(m_hubDistance));
        break;
      case LEFT_FERRY:
        m_LeftFerryDistance = currentTranslation2d.getDistance(DriveConstants.getLeftFerryPose().getTranslation());
        setShooterVelocity(ShooterConstants.getShooterFerryVelocity(m_LeftFerryDistance));
        break;
      case RIGHT_FERRY:
        m_RightFerryDistance = currentTranslation2d.getDistance(DriveConstants.getRightFerryPose().getTranslation());
        setShooterVelocity(ShooterConstants.getShooterFerryVelocity(m_RightFerryDistance));
        break;
      case IDLE:
        setShooterVelocity(ShooterConstants.kPrepSpeed);
        break;
      case STOP:
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
        break;  
    }
  }


  public void setShooterVelocity(double velocity) {
    shooterMotor1.setControl(m_motionRequest.withVelocity(velocity));
  }

  public boolean isAtSetpoint() {
    return Math.abs(shooterMotor1.getVelocity().getValueAsDouble() - m_motionRequest.Velocity) <= ShooterConstants.kVelocityTolerance;
  }
  
  private void logMotorData() {
    Logger.recordOutput("Subsystems/Shooter/ShooterState", currentState.name());

    Logger.recordOutput("Subsystems/Shooter/Velocity/ShooterMotorVelocity", shooterMotor1.getVelocity().getValueAsDouble());
    Logger.recordOutput("Subsystems/Shooter/Velocity/ShooterSetpoint", m_motionRequest.Velocity);
    Logger.recordOutput("Subsystems/Shooter/Velocity/IsAtSetpoint", Math.abs(shooterMotor1.getVelocity().getValueAsDouble() - m_motionRequest.Velocity) <= ShooterConstants.kVelocityTolerance);

    Logger.recordOutput("Subsystems/Shooter/Basic/ShooterMotorSupplyCurrent", shooterMotor1.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Shooter/Basic/ShooterMotorStatorCurrent", shooterMotor1.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Shooter/Basic/ShooterMotorVoltage", shooterMotor1.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Subsystems/Shooter/Tracking/HubDistance", m_hubDistance);
    Logger.recordOutput("Subsystems/Shooter/Tracking/LeftFerryDistance", m_LeftFerryDistance);
    Logger.recordOutput("Subsystems/Shooter/Tracking/RightFerryDistance", m_RightFerryDistance);
  }
}