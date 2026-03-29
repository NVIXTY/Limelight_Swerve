// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.DriveConstants;
import frc.robot.util.ShotCalculator;

public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final TalonFXConfiguration shooterConfig;

    private VelocityVoltage m_velocityRequest;

    private ShooterState currentState = ShooterState.STOP;

    private double m_hubDistance = 0.0;
    private double m_leftFerryDistance = 0.0;
    private double m_rightFerryDistance = 0.0;

    private double m_nextRelookTime = 0.0;
    /** Seconds between distance→setpoint refreshes while shooting (lower = tracks moving targets faster). */
    private static final double kRelookDefault = 0.05;
    private double m_lastLogTime = 0.0;
    private static final double kLogPeriod = 0.10;

    public Shooter() {
        shooterMotor1 = new TalonFX(ShooterConstants.kShooterMotorId1);
        shooterMotor2 = new TalonFX(ShooterConstants.kShooterMotorId2);

        shooterConfig = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withVelocityFilterTimeConstant(ShooterConstants.kVelocityFilterTimeSec))
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
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(ShooterConstants.kSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(ShooterConstants.kStatorCurrentLimit))
                .withStatorCurrentLimitEnable(false));
        shooterMotor1.getConfigurator().apply(shooterConfig);

        shooterMotor2.setControl(new Follower(ShooterConstants.kShooterMotorId1, MotorAlignmentValue.Opposed));
        shooterMotor2.getConfigurator().apply(shooterConfig);

        m_velocityRequest = new VelocityVoltage(0).withSlot(0);

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

        Logger.recordOutput("Subsystems/Shooter/Tracking/HubDistance", m_hubDistance);
        Logger.recordOutput("Subsystems/Shooter/Tracking/LeftFerryDistance", m_leftFerryDistance);
        Logger.recordOutput("Subsystems/Shooter/Tracking/RightFerryDistance", m_rightFerryDistance);

        now = Timer.getFPGATimestamp();
        double relook = SmartDashboard.getNumber("Shooter/RelookSeconds", kRelookDefault);
        if (relook <= 0) {
            relook = kRelookDefault;
        }
        if (now >= m_nextRelookTime + relook) {
            switch (currentState) {
                case HUB:
                case LEFT_FERRY:
                case RIGHT_FERRY:
                    setGoal(currentState);
                    break;
                case IDLE:
                    shooterMotor1.setControl(new CoastOut());
                    shooterMotor2.setControl(new CoastOut());
                    break;
                case OUTSHOOT:
                case OUTSHOOT_SLOW:
                case STOP:
                default:
                    break;
            }
            m_nextRelookTime = now;
        }
    }

    public void setGoal(ShooterState desiredState) {
        currentState = desiredState;
        ShotCalculator shotCalc = ShotCalculator.getInstance();
        switch (desiredState) {
            case HUB:
                m_hubDistance = shotCalc.getCompensatedDistance(DriveConstants.getHubPose());
                setShooterVelocity(ShooterConstants.getShooterHubVelocity(m_hubDistance));
                try {
                    NetworkTableInstance.getDefault()
                        .getEntry("AdvantageKit/Subsystems/Shooter/HubDistance")
                        .setDouble(m_hubDistance);
                } catch (Exception ex) {
                    // ignore NT failures
                }
                break;
            case LEFT_FERRY:
                m_leftFerryDistance = shotCalc.getCompensatedDistance(DriveConstants.getLeftFerryPose());
                setShooterVelocity(ShooterConstants.getShooterFerryVelocity(m_leftFerryDistance));
                break;
            case RIGHT_FERRY:
                m_rightFerryDistance = shotCalc.getCompensatedDistance(DriveConstants.getRightFerryPose());
                setShooterVelocity(ShooterConstants.getShooterFerryVelocity(m_rightFerryDistance));
                break;
            case IDLE:
                setShooterVelocity(ShooterConstants.kPrepSpeed);
                break;
            case OUTSHOOT:
                setShooterVelocity(ShooterConstants.kOutshootVelocity);
                break;
            case OUTSHOOT_SLOW:
                setShooterVelocity(ShooterConstants.kOutshootSlowVelocity);
                break;
            case STOP:
                shooterMotor1.stopMotor();
                shooterMotor2.stopMotor();
                break;
        }
    }

    public ShooterState getCurrentState() {
        return currentState;
    }

    public void setShooterVelocity(double velocity) {
        m_velocityRequest = m_velocityRequest.withVelocity(velocity);
        shooterMotor1.setControl(m_velocityRequest);
    }

    public boolean isAtSetpoint() {
        return Math.abs(shooterMotor1.getVelocity().getValueAsDouble() - m_velocityRequest.Velocity)
            <= ShooterConstants.kVelocityTolerance;
    }

    private void logMotorData() {
        Logger.recordOutput("Subsystems/Shooter/ShooterState", currentState.name());

        Logger.recordOutput("Subsystems/Shooter/Velocity/ShooterMotorVelocity", shooterMotor1.getVelocity().getValueAsDouble());
        Logger.recordOutput("Subsystems/Shooter/Velocity/ShooterSetpoint", m_velocityRequest.Velocity);
        Logger.recordOutput(
            "Subsystems/Shooter/Velocity/IsAtSetpoint",
            Math.abs(shooterMotor1.getVelocity().getValueAsDouble() - m_velocityRequest.Velocity)
                <= ShooterConstants.kVelocityTolerance);

        Logger.recordOutput("Subsystems/Shooter/Basic/ShooterMotorSupplyCurrent", shooterMotor1.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Subsystems/Shooter/Basic/ShooterMotorStatorCurrent", shooterMotor1.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Subsystems/Shooter/Basic/ShooterMotorVoltage", shooterMotor1.getMotorVoltage().getValueAsDouble());
    }
}
