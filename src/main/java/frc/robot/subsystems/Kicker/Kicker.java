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

    private enum KurPhase {
        NONE,
        FORWARD,
        REVERSE
    }

    private final TalonFX kickerMotor;
    private final TalonFXConfiguration kickerConfig;

    private KickerState currentState = KickerState.STOP;
    private KurPhase kurPhase = KurPhase.NONE;

    private double kickUntilResistanceT0 = 0.0;
    private int resistanceDebounce = 0;

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
        if (currentState == KickerState.KICK_UNTIL_RESISTANCE && kurPhase == KurPhase.FORWARD) {
            double now = Timer.getFPGATimestamp();
            double elapsed = now - kickUntilResistanceT0;

            if (elapsed >= KickerConstants.kKickUntilResistanceSpinUpSeconds) {
                double stator = kickerMotor.getStatorCurrent().getValueAsDouble();
                Logger.recordOutput("Subsystems/Kicker/KickUntilResistance/StatorA", stator);

                if (stator >= KickerConstants.kKickUntilResistanceStatorThresholdAmps) {
                    resistanceDebounce++;
                    if (resistanceDebounce >= KickerConstants.kKickUntilResistanceDebounceLoops) {
                        Logger.recordOutput("Subsystems/Kicker/KickUntilResistance/EndedReason", "resistance");
                        beginReverseAfterResistance();
                    }
                } else {
                    resistanceDebounce = 0;
                }
            }

            if (elapsed >= KickerConstants.kKickUntilResistanceTimeoutSeconds) {
                Logger.recordOutput("Subsystems/Kicker/KickUntilResistance/EndedReason", "timeout");
                finishKickUntilResistance();
            }
        } else if (currentState == KickerState.KICK_UNTIL_RESISTANCE && kurPhase == KurPhase.REVERSE) {
            double pos = kickerMotor.getPosition().getValueAsDouble();
            double err = Math.abs(pos - reverseTargetRot);
            Logger.recordOutput("Subsystems/Kicker/KickUntilResistance/ReverseErr", err);

            if (err <= KickerConstants.kReversePositionToleranceRot) {
                Logger.recordOutput("Subsystems/Kicker/KickUntilResistance/EndedReason", "reverse_done");
                finishKickUntilResistance();
            } else if (Timer.getFPGATimestamp() - reverseT0 >= KickerConstants.kReverseTimeoutSeconds) {
                Logger.recordOutput("Subsystems/Kicker/KickUntilResistance/EndedReason", "reverse_timeout");
                finishKickUntilResistance();
            }
        }

        logMotorData();
    }

    private void beginReverseAfterResistance() {
        kurPhase = KurPhase.REVERSE;
        reverseT0 = Timer.getFPGATimestamp();
        double pos = kickerMotor.getPosition().getValueAsDouble();
        // Opposite of intake kick: unwind by N rotor rotations (flip sign on robot if it runs wrong way)
        reverseTargetRot = pos - KickerConstants.kAfterResistanceReverseRotations;
        m_reverseRequest = new MotionMagicVoltage(reverseTargetRot).withSlot(0);
        kickerMotor.setControl(m_reverseRequest);
        Logger.recordOutput("Subsystems/Kicker/KickUntilResistance/ReverseTargetRot", reverseTargetRot);
    }

    private void finishKickUntilResistance() {
        kurPhase = KurPhase.NONE;
        resistanceDebounce = 0;
        currentState = KickerState.STOP;
        kickerMotor.stopMotor();
    }

    /** Stops kick-until-resistance (forward or reverse). Safe to call anytime. */
    public void stopKickUntilResistance() {
        if (currentState != KickerState.KICK_UNTIL_RESISTANCE) {
            return;
        }
        Logger.recordOutput("Subsystems/Kicker/KickUntilResistance/EndedReason", "cancelled");
        finishKickUntilResistance();
    }

    public void setGoal(KickerState desiredState) {
        if (desiredState != KickerState.KICK_UNTIL_RESISTANCE && currentState == KickerState.KICK_UNTIL_RESISTANCE) {
            kurPhase = KurPhase.NONE;
            resistanceDebounce = 0;
        }

        currentState = desiredState;
        switch (desiredState) {
            case KICK:
                kurPhase = KurPhase.NONE;
                kickerMotor.set(KickerConstants.kKickerInSpeed);
                break;
            case OUTKICK:
                kurPhase = KurPhase.NONE;
                kickerMotor.set(KickerConstants.kKickerOutSpeed);
                break;
            case KICK_UNTIL_RESISTANCE:
                kurPhase = KurPhase.FORWARD;
                resistanceDebounce = 0;
                kickUntilResistanceT0 = Timer.getFPGATimestamp();
                kickerMotor.set(KickerConstants.kKickerInSpeed);
                break;
            case STOP:
                kurPhase = KurPhase.NONE;
                kickerMotor.stopMotor();
                break;
        }
    }

    private void logMotorData() {
        Logger.recordOutput("Subsystems/Kicker/KickerState", currentState.name());
        Logger.recordOutput("Subsystems/Kicker/KurPhase", kurPhase.name());
        Logger.recordOutput("Subsystems/Kicker/Basic/KickerMotorSpeed", kickerMotor.get());
        Logger.recordOutput("Subsystems/Kicker/Basic/KickerMotorSupplyCurrent", kickerMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Subsystems/Kicker/Basic/KickerMotorStatorCurrent", kickerMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Subsystems/Kicker/Basic/KickerMotorVoltage", kickerMotor.getMotorVoltage().getValueAsDouble());
    }
}
