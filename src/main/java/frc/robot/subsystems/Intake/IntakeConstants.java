package frc.robot.subsystems.Intake;

public final class IntakeConstants {
    private IntakeConstants() {}

    // Hardware
    public static final int MOTOR_ID = 30; // TODO: set real CAN ID
    public static final String CAN_BUS = ""; // default CAN bus
    public static final int ROLLER_MOTOR_ID = 31; // TODO set actual ID

    // Setpoints (motor rotations)
    public static final double TOP_POSITION_ROT = 0.0;
    public static final double DEPLOYED_POSITION_ROT = 20.0; // TODO: tune from dashboard reading

    // PID (slot 0)
    public static final double KP = 8.0;
    public static final double KI = 0.0;
    public static final double KD = 0.2;

    // Roller PID (slot 0)
    public static final double ROLLER_KP = 0.12;
    public static final double ROLLER_KI = 0.0;
    public static final double ROLLER_KD = 0.0;

    // Safety / behavior
    public static final double MANUAL_OUTPUT_LIMIT = 1.0; // clamp [-1, 1]

    // Dashboard keys
    public static final String KEY_POS_ROT = "Intake/PositionRot";
    public static final String KEY_DEPLOYED_SETPOINT = "Intake/DeployedSetpointRot";
    public static final String KEY_KP = "Intake/kP";
    public static final String KEY_KI = "Intake/kI";
    public static final String KEY_KD = "Intake/kD";
    public static final String KEY_ROLLER_FWD_RPS = "Intake/RollerForwardRPS";
    public static final String KEY_ROLLER_REV_RPS = "Intake/RollerReverseRPS";
    public static final String KEY_ROLLER_VEL_RPS = "Intake/RollerVelocityRPS";
}