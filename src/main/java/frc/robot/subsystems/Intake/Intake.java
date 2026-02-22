package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX motor;         // slapdown pivot
    private final TalonFX rollerMotor;   // intake roller (Falcon 500)

    private double deployedPositionRotations = IntakeConstants.DEPLOYED_POSITION_ROT;

    private double kP = IntakeConstants.KP;
    private double kI = IntakeConstants.KI;
    private double kD = IntakeConstants.KD;

    // Roller velocity config (RPS)
    private double rollerForwardVelocityRps = IntakeConstants.ROLLER_FORWARD_VELOCITY_RPS;
    private double rollerReverseVelocityRps = IntakeConstants.ROLLER_REVERSE_VELOCITY_RPS;

    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    private final DutyCycleOut manualRequest = new DutyCycleOut(0);
    private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0).withSlot(0);

    public Intake() {
        this(IntakeConstants.MOTOR_ID, IntakeConstants.ROLLER_MOTOR_ID);
    }

    public Intake(int pivotMotorID) {
        this(pivotMotorID, IntakeConstants.ROLLER_MOTOR_ID);
    }

    public Intake(int pivotMotorID, int rollerMotorID) {
        motor = IntakeConstants.CAN_BUS.isEmpty() ? new TalonFX(pivotMotorID) : new TalonFX(pivotMotorID, IntakeConstants.CAN_BUS);
        rollerMotor = IntakeConstants.CAN_BUS.isEmpty() ? new TalonFX(rollerMotorID) : new TalonFX(rollerMotorID, IntakeConstants.CAN_BUS);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        Slot0Configs pivotSlot0 = pivotConfig.Slot0;
        pivotSlot0.kP = kP;
        pivotSlot0.kI = kI;
        pivotSlot0.kD = kD;
        motor.getConfigurator().apply(pivotConfig);
        motor.setPosition(IntakeConstants.TOP_POSITION_ROT);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        Slot0Configs rollerSlot0 = rollerConfig.Slot0;
        rollerSlot0.kP = IntakeConstants.ROLLER_KP;
        rollerSlot0.kI = IntakeConstants.ROLLER_KI;
        rollerSlot0.kD = IntakeConstants.ROLLER_KD;
        rollerMotor.getConfigurator().apply(rollerConfig);

        SmartDashboard.putNumber(IntakeConstants.KEY_DEPLOYED_SETPOINT, deployedPositionRotations);
        SmartDashboard.putNumber(IntakeConstants.KEY_KP, kP);
        SmartDashboard.putNumber(IntakeConstants.KEY_KI, kI);
        SmartDashboard.putNumber(IntakeConstants.KEY_KD, kD);
        SmartDashboard.putNumber(IntakeConstants.KEY_ROLLER_FWD_RPS, rollerForwardVelocityRps);
        SmartDashboard.putNumber(IntakeConstants.KEY_ROLLER_REV_RPS, rollerReverseVelocityRps);
    }

    /** Moves intake to deployed setpoint and spins roller forward at configured velocity. */
    public void deploy() {
        deployedPositionRotations = SmartDashboard.getNumber(
            IntakeConstants.KEY_DEPLOYED_SETPOINT, deployedPositionRotations);
        motor.setControl(positionRequest.withPosition(deployedPositionRotations));
        runRollerForward();
    }

    /** Moves intake to top (0 rotations) and stops roller. */
    public void retract() {
        motor.setControl(positionRequest.withPosition(IntakeConstants.TOP_POSITION_ROT));
        stopRoller();
    }

    /** Manual open-loop pivot control for tuning (-1.0 to 1.0). */
    public void manualControl(double percentOutput) {
        double clamped = MathUtil.clamp(
            percentOutput,
            -IntakeConstants.MANUAL_OUTPUT_LIMIT,
            IntakeConstants.MANUAL_OUTPUT_LIMIT);
        motor.setControl(manualRequest.withOutput(clamped));
    }

    /** Runs roller forward in velocity closed-loop. */
    public void runRollerForward() {
        rollerForwardVelocityRps = SmartDashboard.getNumber(IntakeConstants.KEY_ROLLER_FWD_RPS, rollerForwardVelocityRps);
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(rollerForwardVelocityRps));
    }

    /** Runs roller reverse in velocity closed-loop. */
    public void runRollerReverse() {
        rollerReverseVelocityRps = SmartDashboard.getNumber(IntakeConstants.KEY_ROLLER_REV_RPS, rollerReverseVelocityRps);
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(-Math.abs(rollerReverseVelocityRps)));
    }

    /** Stops only roller motor. */
    public void stopRoller() {
        rollerMotor.stopMotor();
    }

    /** Stops both pivot and roller motors. */
    public void stop() {
        motor.stopMotor();
        rollerMotor.stopMotor();
    }

    public void setDeployedPositionRotations(double rotations) {
        deployedPositionRotations = rotations;
        SmartDashboard.putNumber(IntakeConstants.KEY_DEPLOYED_SETPOINT, deployedPositionRotations);
    }

    public double getPositionRotations() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getRollerVelocityRps() {
        return rollerMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(IntakeConstants.KEY_POS_ROT, getPositionRotations());
        SmartDashboard.putNumber(IntakeConstants.KEY_ROLLER_VEL_RPS, getRollerVelocityRps());

        double newKP = SmartDashboard.getNumber(IntakeConstants.KEY_KP, kP);
        double newKI = SmartDashboard.getNumber(IntakeConstants.KEY_KI, kI);
        double newKD = SmartDashboard.getNumber(IntakeConstants.KEY_KD, kD);

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
