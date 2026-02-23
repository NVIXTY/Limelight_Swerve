package frc.robot.subsystems.Index;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
    private final TalonFX motor;
    private final TalonFX kickerMotor;

    private final VelocityVoltage indexVelocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage kickerVelocityRequest = new VelocityVoltage(0).withSlot(0);

    private double indexVelocity = IndexConstants.TARGET_VELOCITY_RPS;
    private double reverseVelocity = Math.abs(IndexConstants.REVERSE_VELOCITY_RPS);

    private double kickerVelocity = IndexConstants.KICKER_TARGET_VELOCITY_RPS;
    private double kickerReverseVelocity = Math.abs(IndexConstants.KICKER_REVERSE_VELOCITY_RPS);

    private double kP = IndexConstants.KP;
    private double kI = IndexConstants.KI;
    private double kD = IndexConstants.KD;

    public Index() {
        this(IndexConstants.MOTOR_ID, IndexConstants.KICKER_MOTOR_ID);
    }

    public Index(int motorID) {
        this(motorID, IndexConstants.KICKER_MOTOR_ID);
    }

    public Index(int motorID, int kickerMotorID) {
        motor = IndexConstants.CAN_BUS.isEmpty() ? new TalonFX(motorID) : new TalonFX(motorID, IndexConstants.CAN_BUS);
        kickerMotor = IndexConstants.CAN_BUS.isEmpty() ? new TalonFX(kickerMotorID) : new TalonFX(kickerMotorID, IndexConstants.CAN_BUS);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = IndexConstants.NEUTRAL_MODE;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;

        motor.getConfigurator().apply(config);
        kickerMotor.getConfigurator().apply(config);

        SmartDashboard.putNumber(IndexConstants.KEY_TARGET_VELOCITY, indexVelocity);
        SmartDashboard.putNumber(IndexConstants.KEY_REVERSE_VELOCITY, reverseVelocity);
        SmartDashboard.putNumber(IndexConstants.KEY_KICKER_TARGET_VELOCITY, kickerVelocity);
        SmartDashboard.putNumber(IndexConstants.KEY_KICKER_REVERSE_VELOCITY, kickerReverseVelocity);

        SmartDashboard.putNumber(IndexConstants.KEY_KP, kP);
        SmartDashboard.putNumber(IndexConstants.KEY_KI, kI);
        SmartDashboard.putNumber(IndexConstants.KEY_KD, kD);
    }

    /** Runs index + kicker forward at their configured velocities. */
    public void runIndex() {
        indexVelocity = Math.abs(SmartDashboard.getNumber(IndexConstants.KEY_TARGET_VELOCITY, indexVelocity));
        kickerVelocity = Math.abs(SmartDashboard.getNumber(IndexConstants.KEY_KICKER_TARGET_VELOCITY, kickerVelocity));

        motor.setControl(indexVelocityRequest.withVelocity(indexVelocity));
        kickerMotor.setControl(kickerVelocityRequest.withVelocity(kickerVelocity));
    }

    /** Runs index + kicker reverse at their configured velocities. */
    public void runIndexReverse() {
        reverseVelocity = Math.abs(SmartDashboard.getNumber(IndexConstants.KEY_REVERSE_VELOCITY, reverseVelocity));
        kickerReverseVelocity = Math.abs(SmartDashboard.getNumber(IndexConstants.KEY_KICKER_REVERSE_VELOCITY, kickerReverseVelocity));

        motor.setControl(indexVelocityRequest.withVelocity(-reverseVelocity));
        kickerMotor.setControl(kickerVelocityRequest.withVelocity(-kickerReverseVelocity));
    }

    /** Stops the index and kicker motors. */
    public void stop() {
        motor.stopMotor();
        kickerMotor.stopMotor();
    }

    public double getVelocityRps() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getKickerVelocityRps() {
        return kickerMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(IndexConstants.KEY_VELOCITY, getVelocityRps());
        SmartDashboard.putNumber(IndexConstants.KEY_KICKER_VELOCITY, getKickerVelocityRps());

        double newKP = SmartDashboard.getNumber(IndexConstants.KEY_KP, kP);
        double newKI = SmartDashboard.getNumber(IndexConstants.KEY_KI, kI);
        double newKD = SmartDashboard.getNumber(IndexConstants.KEY_KD, kD);

        if (newKP != kP || newKI != kI || newKD != kD) {
            kP = newKP;
            kI = newKI;
            kD = newKD;

            Slot0Configs updated = new Slot0Configs();
            updated.kP = kP;
            updated.kI = kI;
            updated.kD = kD;

            motor.getConfigurator().apply(updated);
            kickerMotor.getConfigurator().apply(updated);
        }
    }
}
