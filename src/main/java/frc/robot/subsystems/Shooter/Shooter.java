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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;

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

  
  private boolean m_usingHighAccel = false;
  private double m_highAccelExpireTime = 0.0;
  private static final double kHighAccelMultiplier = 3.0; // how much faster to ramp
  private static final double kHighAccelDuration = 0.25; // seconds to keep high accel
  private static final double kRecoverVelocityStep = 6.0; // RPS threshold to trigger rapid recover
  private TalonFXConfiguration m_highAccelConfig;
  private double m_nextRelookTime = 0.0;
  private static final double kRelookDefault = 0.125; // seconds
  private double m_lastLogTime = 0.0;
  private static final double kLogPeriod = 0.10; // seconds (10 Hz)

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

    
    m_highAccelConfig = new TalonFXConfiguration()
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
                                    .withMotionMagicAcceleration((int)(ShooterConstants.kAcceleration * kHighAccelMultiplier))
                                    .withMotionMagicJerk((int)(ShooterConstants.kJerk * kHighAccelMultiplier)))
                    .withCurrentLimits(new CurrentLimitsConfigs()
                                    .withSupplyCurrentLimit(ShooterConstants.kSupplyCurrentLimit));

  
  SmartDashboard.putNumber("Shooter/RelookSeconds", kRelookDefault);
  m_nextRelookTime = Timer.getFPGATimestamp();

  }
  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    if (now - m_lastLogTime >= kLogPeriod) {
      logMotorData();
      m_lastLogTime = now;
    }

    // Publish tracking distances every periodic (unthrottled) so Junction Logger
    // has the latest values for AdvantageKit via NT4Publisher.
    Logger.recordOutput("Subsystems/Shooter/Tracking/HubDistance", m_hubDistance);
    Logger.recordOutput("Subsystems/Shooter/Tracking/LeftFerryDistance", m_LeftFerryDistance);
    Logger.recordOutput("Subsystems/Shooter/Tracking/RightFerryDistance", m_RightFerryDistance);

    // Restore normal motion magic configs after the high-accel window elapses
    if (m_usingHighAccel && Timer.getFPGATimestamp() > m_highAccelExpireTime) {
      try {
        shooterMotor1.getConfigurator().apply(shooterConfig);
        shooterMotor2.getConfigurator().apply(shooterConfig);
      } catch (Exception ex) {
        // ignore
      }
      m_usingHighAccel = false;
    }

  // Periodically re-evaluate distances and adjust shooter velocity when in a shooting state.
  now = Timer.getFPGATimestamp();
    double relook = SmartDashboard.getNumber("Shooter/RelookSeconds", kRelookDefault);
    if (relook <= 0) relook = kRelookDefault;
    if (now >= m_nextRelookTime + relook) {
      // Only recompute when we are in a state that uses distances or a prep speed
      switch (currentState) {
        case HUB:
        case LEFT_FERRY:
        case RIGHT_FERRY:
        case IDLE:
          // Reuse setGoal logic to recompute distances and set velocities.
          setGoal(currentState);
          break;
        default:
          // STOP or other states: no periodic re-eval
          break;
      }
      m_nextRelookTime = now;
    }
  }

  public void setGoal(ShooterState desiredState) {
    currentState = desiredState;
    Translation2d currentTranslation2d = m_swerveSubsystem.getState().Pose.getTranslation();
    switch (desiredState) {
      case HUB:
        m_hubDistance = currentTranslation2d.getDistance(frc.robot.subsystems.Drive.AutoAim.getVirtualHubPose(m_swerveSubsystem.getState().Pose, m_swerveSubsystem.getState().Speeds).getTranslation());
        setShooterVelocity(ShooterConstants.getShooterHubVelocity(m_hubDistance));
        // Publish hub distance to NetworkTables for AdvantageKit / NT consumers
        try {
          NetworkTableInstance.getDefault().getEntry("AdvantageKit/Subsystems/Shooter/HubDistance").setDouble(m_hubDistance);
        } catch (Exception ex) {
          // ignore NT failures
        }
        break;
      case LEFT_FERRY:
        m_LeftFerryDistance = currentTranslation2d.getDistance(frc.robot.subsystems.Drive.AutoAim.getVirtualLeftFerryPose(m_swerveSubsystem.getState().Pose, m_swerveSubsystem.getState().Speeds).getTranslation());
        setShooterVelocity(ShooterConstants.getShooterFerryVelocity(m_LeftFerryDistance));
        break;
      case RIGHT_FERRY:
        m_RightFerryDistance = currentTranslation2d.getDistance(frc.robot.subsystems.Drive.AutoAim.getVirtualRightFerryPose(m_swerveSubsystem.getState().Pose, m_swerveSubsystem.getState().Speeds).getTranslation());
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
    double currentVel = shooterMotor1.getVelocity().getValueAsDouble();
    // If this is a large positive step, temporarily bump accel/jerk for faster recovery
    if (velocity - currentVel > kRecoverVelocityStep) {
      try {
        shooterMotor1.getConfigurator().apply(m_highAccelConfig);
        shooterMotor2.getConfigurator().apply(m_highAccelConfig);
      } catch (Exception ex) {
        // ignore configurator failures; just fall back to normal behavior
      }
      m_usingHighAccel = true;
      m_highAccelExpireTime = Timer.getFPGATimestamp() + kHighAccelDuration;
    }

    m_motionRequest = m_motionRequest.withVelocity(velocity);
    shooterMotor1.setControl(m_motionRequest);
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

    // Note: tracking distances are recorded every periodic (unthrottled) so AdvantageKit
    // receives the latest values via Junction/NT4. Keep other motor telemetry throttled.
  }

}