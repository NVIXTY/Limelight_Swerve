package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ClimbConstants {
    private ClimbConstants() {}

    // Hardware
    public static final int MOTOR_ID = 50; // set your real CAN ID
    public static final String CAN_BUS = ""; // default bus

    // Setpoints (rotations)
    public static final double STOWED_POSITION_ROT = 0.0;
    public static final double DEPLOYED_POSITION_ROT = 30.0;

    // PID
    public static final double KP = 8.0;
    public static final double KI = 0.0;
    public static final double KD = 0.2;

    // Motor behavior
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double MANUAL_OUTPUT_LIMIT = 1.0;

    // Dashboard keys
    public static final String KEY_POSITION_ROT = "Climb/PositionRot";
    public static final String KEY_STOWED_ROT = "Climb/StowedRot";
    public static final String KEY_DEPLOYED_ROT = "Climb/DeployedRot";
    public static final String KEY_KP = "Climb/kP";
    public static final String KEY_KI = "Climb/kI";
    public static final String KEY_KD = "Climb/kD";
}