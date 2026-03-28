// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {

    private enum SlowKickPhase {
        NONE,
        FORWARD,
        REVERSE
    }

    private final TalonFX kickerMotor;
    private final TalonFXConfiguration kickerConfig;

    private KickerState currentState = KickerState.STOP;
    private SlowKickPhase slowKickPhase = SlowKickPhase.NONE;

    private double reverseT0 = 0.0;
    private double reverseTargetRot = 0.0;
    private MotionMagicVoltage m_reverseRequest = new MotionMagicVoltage(0).withSlot(0);

    public Kicker() {
        kickerMotor = new TalonFX(KickerConstants.kKickerMotorId);

        kickerConfig = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs())
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs()
                .withKP(KickerConstants.kReversePositionkP)
                .withKI(KickerConstants.kReversePositionkI)
                .withKD(KickerConstants.kReversePositionkD))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(KickerConstants.kReverseMMCruiseRps)
                .withMotionMagicAcceleration(KickerConstants.kReverseMMAccelRps2))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(KickerConstants.kKickerSupplyCurrentLimit));

        kickerMotor.getConfigurator().apply(kickerConfig);
    }

    @Override
    public void periodic() {
        if (currentState == KickerState.SLOW_KICK && slowKickPhase == SlowKickPhase.REVERSE) {
            double pos = kickerMotor.getPosition().getValueAsDouble();
            double err = Math.abs(pos - reverseTargetRot);
            Logger.recordOutput("Subsystems/Kicker/SlowKick/ReverseErr", err);

            if (err <= KickerConstants.kReversePositionToleranceRot) {
                Logger.recordOutput("Subsystems/Kicker/SlowKick/EndedReason", "reverse_done");
                finishSlowKickReverse();
            } else if (Timer.getFPGATimestamp() - reverseT0 >= KickerConstants.kReverseTimeoutSeconds) {
                Logger.recordOutput("Subsystems/Kicker/SlowKick/EndedReason", "reverse_timeout");
                finishSlowKickReverse();
            }
        }

        logMotorData();
    }

    private void beginReverseAfterSlowKick() {
        slowKickPhase = SlowKickPhase.REVERSE;
        reverseT0 = Timer.getFPGATimestamp();
        double pos = kickerMotor.getPosition().getValueAsDouble();
        reverseTargetRot = pos - KickerConstants.kSlowKickReverseRotations;
        m_reverseRequest = new MotionMagicVoltage(reverseTargetRot).withSlot(0);
        kickerMotor.setControl(m_reverseRequest);
        Logger.recordOutput("Subsystems/Kicker/SlowKick/ReverseTargetRot", reverseTargetRot);
    }

    private void finishSlowKickReverse() {
        slowKickPhase = SlowKickPhase.NONE;
        currentState = KickerState.STOP;
        kickerMotor.stopMotor();
    }

    /**
     * Release slow kick: if still feeding forward, run reverse unwind; if already reversing, no-op.
     */
    public void releaseSlowKick() {
        if (currentState != KickerState.SLOW_KICK) {
            return;
        }
        if (slowKickPhase == SlowKickPhase.FORWARD) {
            beginReverseAfterSlowKick();
        }
    }

    /** Abort slow kick (forward or reverse) immediately — e.g. hub/ferry takes over. */
    public void cancelSlowKick() {
        if (currentState != KickerState.SLOW_KICK) {
            return;
        }
        Logger.recordOutput("Subsystems/Kicker/SlowKick/EndedReason", "cancelled");
        slowKickPhase = SlowKickPhase.NONE;
        currentState = KickerState.STOP;
        kickerMotor.stopMotor();
    }

    public void setGoal(KickerState desiredState) {
        if (currentState == KickerState.SLOW_KICK && desiredState != KickerState.SLOW_KICK) {
            if (desiredState == KickerState.STOP) {
                releaseSlowKick();
                return;
            }
            cancelSlowKick();
        }

        currentState = desiredState;
        switch (desiredState) {
            case KICK:
                slowKickPhase = SlowKickPhase.NONE;
                kickerMotor.set(KickerConstants.kKickerInSpeed);
                break;
            case OUTKICK:
                slowKickPhase = SlowKickPhase.NONE;
                kickerMotor.set(KickerConstants.kKickerOutSpeed);
                break;
            case SLOW_KICK:
                slowKickPhase = SlowKickPhase.FORWARD;
                kickerMotor.set(KickerConstants.kKickerInSpeed * KickerConstants.kKickerSlowKickMultiplier);
                break;
            case STOP:
                slowKickPhase = SlowKickPhase.NONE;
                kickerMotor.stopMotor();
                break;
        }
    }

    private void logMotorData() {
        Logger.recordOutput("Subsystems/Kicker/KickerState", currentState.name());
        Logger.recordOutput("Subsystems/Kicker/SlowKickPhase", slowKickPhase.name());
        Logger.recordOutput("Subsystems/Kicker/Basic/KickerMotorSpeed", kickerMotor.get());
        Logger.recordOutput("Subsystems/Kicker/Basic/KickerMotorSupplyCurrent", kickerMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Subsystems/Kicker/Basic/KickerMotorStatorCurrent", kickerMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Subsystems/Kicker/Basic/KickerMotorVoltage", kickerMotor.getMotorVoltage().getValueAsDouble());
    }
}
