package frc.robot.subsystems.Index;

import com.ctre.phoenix6.signals.NeutralModeValue;

public final class IndexConstants {
    private IndexConstants() {}

    // Hardware
    public static final int MOTOR_ID = 40;            // index motor CAN ID
    public static final int KICKER_MOTOR_ID = 41;     // kicker motor CAN ID
    public static final String CAN_BUS = "";           // default bus

    // Control defaults (RPS)
    public static final double TARGET_VELOCITY_RPS = 20.0;
    public static final double REVERSE_VELOCITY_RPS = 10.0;

    // Kicker velocities (RPS)
    public static final double KICKER_TARGET_VELOCITY_RPS = 30.0;
    public static final double KICKER_REVERSE_VELOCITY_RPS = 15.0;

    // PID (slot 0) applied to both motors
    public static final double KP = 0.12;
    public static final double KI = 0.0;
    public static final double KD = 0.0;

    // Motor behavior
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    // Dashboard keys
    public static final String KEY_TARGET_VELOCITY = "Index/TargetVelocityRPS";
    public static final String KEY_REVERSE_VELOCITY = "Index/ReverseVelocityRPS";
    public static final String KEY_VELOCITY = "Index/VelocityRPS";

    public static final String KEY_KICKER_TARGET_VELOCITY = "Index/KickerTargetVelocityRPS";
    public static final String KEY_KICKER_REVERSE_VELOCITY = "Index/KickerReverseVelocityRPS";
    public static final String KEY_KICKER_VELOCITY = "Index/KickerVelocityRPS";

    public static final String KEY_KP = "Index/kP";
    public static final String KEY_KI = "Index/kI";
    public static final String KEY_KD = "Index/kD";
}