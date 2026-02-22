package frc.robot.subsystems.Index;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
    private final TalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private double indexVelocity = IndexConstants.TARGET_VELOCITY_RPS;

    private double kP = IndexConstants.KP;
    private double kI = IndexConstants.KI;
    private double kD = IndexConstants.KD;

    public Index() {
        this(IndexConstants.MOTOR_ID);
    }

    public Index(int motorID) {
        motor = IndexConstants.CAN_BUS.isEmpty() ? new TalonFX(motorID) : new TalonFX(motorID, IndexConstants.CAN_BUS);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = IndexConstants.NEUTRAL_MODE;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;

        motor.getConfigurator().apply(config);

        SmartDashboard.putNumber(IndexConstants.KEY_TARGET_VELOCITY, indexVelocity);
        SmartDashboard.putNumber(IndexConstants.KEY_KP, kP);
        SmartDashboard.putNumber(IndexConstants.KEY_KI, kI);
        SmartDashboard.putNumber(IndexConstants.KEY_KD, kD);
    }

    /** Runs indexer at configured velocity (RPS). */
    public void runIndex() {
        indexVelocity = SmartDashboard.getNumber(IndexConstants.KEY_TARGET_VELOCITY, indexVelocity);
        motor.setControl(velocityRequest.withVelocity(indexVelocity));
    }

    /** Stops the index motor. */
    public void stop() {
        motor.stopMotor();
    }

    /** Optional runtime setter (e.g., from commands). */
    public void setIndexVelocity(double velocityRps) {
        indexVelocity = velocityRps;
        SmartDashboard.putNumber(IndexConstants.KEY_TARGET_VELOCITY, indexVelocity);
    }

    public double getVelocityRps() {
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(IndexConstants.KEY_VELOCITY, getVelocityRps());

        // Optional live PID retune from dashboard
        double newKP = SmartDashboard.getNumber(IndexConstants.KEY_KP, kP);
        double newKI = SmartDashboard.getNumber(IndexConstants.KEY_KI, kI);
        double newKD = SmartDashboard.getNumber(IndexConstants.KEY_KD, kD);

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
