package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMotor;
  private final TalonFXConfiguration shooterConfig;

  private final MotionMagicVelocityVoltage m_motionRequest;
  private final VoltageOut m_voltageRequest;

  @SuppressWarnings("unused")
  private final CommandSwerveDrivetrain m_swerveSubsystem;

  private final InterpolatingDoubleTreeMap kShooterMap = new InterpolatingDoubleTreeMap();

  public Shooter(CommandSwerveDrivetrain swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;
    shooterMotor = new TalonFX(ShooterConstants.MOTOR_ID);

    shooterConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(ShooterConstants.INVERTED)
            .withNeutralMode(ShooterConstants.NEUTRAL_MODE))
        .withSlot0(new Slot0Configs()
            .withKP(ShooterConstants.KP)
            .withKI(ShooterConstants.KI)
            .withKD(ShooterConstants.KD))
        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ShooterConstants.MM_CRUISE_VELOCITY)
            .withMotionMagicAcceleration(ShooterConstants.MM_ACCELERATION)
            .withMotionMagicJerk(ShooterConstants.MM_JERK))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ShooterConstants.SUPPLY_CURRENT_LIMIT));

    shooterMotor.getConfigurator().apply(shooterConfig);

    m_voltageRequest = new VoltageOut(0);
    m_motionRequest = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(true);

    for (double[] point : ShooterConstants.SHOOTER_MAP_POINTS) {
      kShooterMap.put(point[0], point[1]);
    }
  }

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volts.of(ShooterConstants.SYSID_STEP_VOLTS),
          Seconds.of(ShooterConstants.SYSID_TIMEOUT_SECONDS),
          (state) -> SignalLogger.writeString("Shooter State", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> shooterMotor.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
          null,
          this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {}

  public double getShooterVelocityBasedOnDistance(double distanceToGoal) {
    return kShooterMap.get(distanceToGoal);
  }

  public void setShooterVelocity(double velocity) {
    shooterMotor.setControl(m_motionRequest.withVelocity(velocity));
  }

  public void shooterOn() {
    shooterMotor.set(ShooterConstants.SHOOTER_ON_PERCENT);
  }

  public void shooterReverse() {
    shooterMotor.set(ShooterConstants.SHOOTER_REVERSE_PERCENT);
  }

  public void shooterOff() {
    shooterMotor.stopMotor();
  }
}