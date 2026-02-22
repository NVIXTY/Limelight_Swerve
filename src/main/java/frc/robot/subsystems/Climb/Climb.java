package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final TalonFX motor;

    // Two setpoints in motor rotations
    private double stowedPositionRot = 0.0;
    private double deployedPositionRot = 30.0; // tune on robot

    // PID (slot 0)
    private double kP = 8.0;
    private double kI = 0.0;
    private double kD = 0.2;

    // Manual output limit
    private static final double MANUAL_LIMIT = 1.0;

    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    private final DutyCycleOut manualRequest = new DutyCycleOut(0);

    public Climb(int motorID) {
        motor = new TalonFX(motorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;

        motor.getConfigurator().apply(config);

        // Start at stowed reference
        motor.setPosition(stowedPositionRot);

        SmartDashboard.putNumber("Climb/StowedRot", stowedPositionRot);
        SmartDashboard.putNumber("Climb/DeployedRot", deployedPositionRot);
        SmartDashboard.putNumber("Climb/kP", kP);
        SmartDashboard.putNumber("Climb/kI", kI);
        SmartDashboard.putNumber("Climb/kD", kD);
    }

    /** Closed-loop to stowed setpoint. */
    public void retract() {
        stowedPositionRot = SmartDashboard.getNumber("Climb/StowedRot", stowedPositionRot);
        motor.setControl(positionRequest.withPosition(stowedPositionRot));
    }

    /** Closed-loop to deployed setpoint. */
    public void deploy() {
        deployedPositionRot = SmartDashboard.getNumber("Climb/DeployedRot", deployedPositionRot);
        motor.setControl(positionRequest.withPosition(deployedPositionRot));
    }

    /** Manual open-loop control for tuning (-1 to 1). */
    public void manualControl(double percentOutput) {
        double clamped = MathUtil.clamp(percentOutput, -MANUAL_LIMIT, MANUAL_LIMIT);
        motor.setControl(manualRequest.withOutput(clamped));
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getPositionRotations() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb/PositionRot", getPositionRotations());

        // Optional live PID retune
        double newKP = SmartDashboard.getNumber("Climb/kP", kP);
        double newKI = SmartDashboard.getNumber("Climb/kI", kI);
        double newKD = SmartDashboard.getNumber("Climb/kD", kD);

        if (newKP != kP || newKI != kI || newKD != kD) {
            kP = newKP;
            kI = newKI;
            kD = newKD;

            Slot0Configs slot0 = new Slot0Configs();
            slot0.kP = kP;
            slot0.kI = kI;
            slot0.kD = kD;
            motor.getConfigurator().apply(slot0);
        }
    }
}
