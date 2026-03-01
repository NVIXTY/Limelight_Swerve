package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Drive.SwerveDrivetrain;

public class Shooter extends SubsystemBase {
  private TalonFX shooterMotor1;
  private TalonFX shooterMotor2;
  private TalonFXConfiguration shooterConfig;

  private MotionMagicVelocityVoltage m_motionRequest1;
  private MotionMagicVelocityVoltage m_motionRequest2;
  private VoltageOut m_voltageRequest;

<<<<<<< HEAD
  private SwerveDrivetrain m_swerveSubsystem;
=======
  @SuppressWarnings("unused")
  private CommandSwerveDrivetrain m_swerveSubsystem;
>>>>>>> 995b5c5 (cleaned up robotcontainer, added support for dual motor on shooter.)

  private InterpolatingDoubleTreeMap kShooterMap = new InterpolatingDoubleTreeMap();


  public Shooter(SwerveDrivetrain swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;

    shooterMotor1 = new TalonFX(0); // TODO: set correct ID
    shooterMotor2 = new TalonFX(1); // TODO: set correct ID

    shooterConfig = new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs()
                                    .withInverted(InvertedValue.CounterClockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake))
                    .withSlot0(new Slot0Configs()
                              .withKP(1)
                              .withKI(1)
                              .withKD(1))
                    .withMotionMagic(new MotionMagicConfigs()
                                    .withMotionMagicCruiseVelocity(10)
                                    .withMotionMagicAcceleration(160)
                                    .withMotionMagicJerk(1000))
                    .withCurrentLimits(new CurrentLimitsConfigs()
                                    .withSupplyCurrentLimit(35));

    shooterMotor1.getConfigurator().apply(shooterConfig);
    shooterMotor2.getConfigurator().apply(shooterConfig);

    m_voltageRequest = new VoltageOut(0);
    m_motionRequest1 = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(true);
    m_motionRequest2 = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(true);

    kShooterMap.put(1.0, 1.0);
    kShooterMap.put(2.0, 2.1);
    kShooterMap.put(3.0, 3.2);
    kShooterMap.put(4.0, 4.0);
    kShooterMap.put(5.0, 5.0);
  }

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volts.of(4),
          Seconds.of(10),
          (state) -> SignalLogger.writeString("Shooter State", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> {
            shooterMotor1.setControl(m_voltageRequest.withOutput(volts.in(Volts)));
            shooterMotor2.setControl(m_voltageRequest.withOutput(-volts.in(Volts)));
          },
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
    shooterMotor1.setControl(m_motionRequest1.withVelocity(velocity));
    shooterMotor2.setControl(m_motionRequest2.withVelocity(-velocity)); // opposite side of axle
  }

  public void shooterOn() {
    shooterMotor1.set(0.25);
    shooterMotor2.set(-0.25);
  }

  public void shooterReverse() {
    shooterMotor1.set(-0.25);
    shooterMotor2.set(0.25);
  }

  public void shooterOff() {
    shooterMotor1.stopMotor();
    shooterMotor2.stopMotor();
  }
}