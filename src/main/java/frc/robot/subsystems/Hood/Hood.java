// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

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

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.AutoAim;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class Hood extends SubsystemBase {
    private final TalonFX hoodMotor;
    private final TalonFXConfiguration hoodConfig;

    private MotionMagicVoltage m_motionRequest;

    private HoodState currentState = HoodState.STOP;
    private final CommandSwerveDrivetrain m_swerveSubsystem;

    public Hood(CommandSwerveDrivetrain swerveSubsystem) {
        this.m_swerveSubsystem = swerveSubsystem;
        hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
        hoodConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs()
                .withKP(HoodConstants.kP)
                .withKI(HoodConstants.kI)
                .withKD(HoodConstants.kD))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(HoodConstants.kCruiseVelocity)
                .withMotionMagicAcceleration(HoodConstants.kAcceleration))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(HoodConstants.kSupplyCurrentLimit))
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(HoodConstants.kSensorToMechanismRatio));

        hoodMotor.getConfigurator().apply(hoodConfig);

        m_motionRequest = new MotionMagicVoltage(0).withSlot(0);

        hoodMotor.setPosition(0);
    }

    private double hubDistanceMeters() {
        var state = m_swerveSubsystem.getState();
        Translation2d robotT = state.Pose.getTranslation();
        Translation2d hubT = AutoAim.getVirtualHubPose(state.Pose, state.Speeds).getTranslation();
        return robotT.getDistance(hubT);
    }

    public void setGoal(HoodState desiredState) {
        currentState = desiredState;
        switch (desiredState) {
            case TRENCH:
                sethoodPosition(HoodConstants.kTrenchPosition);
                break;
            case HUB:
                if (m_swerveSubsystem != null) {
                    sethoodPosition(HoodConstants.getHoodPosition(hubDistanceMeters()));
                } else {
                    sethoodPosition(HoodConstants.kHubPosition);
                }
                break;
            case FERRY:
                sethoodPosition(HoodConstants.kFerryPosition);
                break;
            case STOP:
                hoodMotor.stopMotor();
                break;
            default:
                break;
        }
    }

    public void hoodUp() {
        hoodMotor.set(HoodConstants.kSpeed);
    }

    public void hoodDown() {
        hoodMotor.set(-HoodConstants.kSpeed);
    }

    public void hoodStop() {
        hoodMotor.stopMotor();
    }

    public void zeroIntake() {
        hoodMotor.setPosition(0);
    }

    public void sethoodPosition(double position) {
        hoodMotor.setControl(m_motionRequest.withPosition(position));
    }

    @Override
    public void periodic() {
        logMotorData();

        try {
            switch (currentState) {
                case HUB:
                    if (m_swerveSubsystem != null) {
                        sethoodPosition(HoodConstants.getHoodPosition(hubDistanceMeters()));
                    }
                    break;
                case FERRY:
                    sethoodPosition(HoodConstants.kFerryPosition);
                    break;
                default:
                    break;
            }
        } catch (Exception ex) {
            // Avoid breaking the main loop from periodic errors.
        }
    }

    public boolean isAtSetpoint() {
        return Math.abs((hoodMotor.getPosition().getValueAsDouble()) - (m_motionRequest.Position))
            <= HoodConstants.kTolerance;
    }

    public void logMotorData() {
        Logger.recordOutput("Subsystems/Hood/HoodState", currentState.name());

        Logger.recordOutput("Subsystems/Hood/Basic/HoodMotorSpeed", hoodMotor.get());
        Logger.recordOutput("Subsystems/Hood/Basic/HoodMotorSupplyCurrent", hoodMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Subsystems/Hood/Basic/HoodMotorStatorCurrent", hoodMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Subsystems/Hood/Basic/HoodMotorVoltage", hoodMotor.getMotorVoltage().getValueAsDouble());

        Logger.recordOutput("Subsystems/Hood/Basic/HoodMotorPosition", hoodMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Subsystems/Hood/Basic/HoodMotorSetpoint", m_motionRequest.Position);
        Logger.recordOutput("Subsystems/Hood/Basic/IsAtSetpoint", isAtSetpoint());
    }
}
