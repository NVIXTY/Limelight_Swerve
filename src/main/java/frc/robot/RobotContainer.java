// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.AutoAim;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodState;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Kicker.KickerState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotContainer {
    private final double MaxSpeed = .75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

    // Drive requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake = new Intake();
    public final Kicker kicker = new Kicker();
    public final Indexer indexer = new Indexer();
    public final Shooter shooter = new Shooter(drivetrain);
    public final Hood hood = new Hood(drivetrain);
    private final AutoAim autoAim = new AutoAim(drivetrain, this);
    
    // Auto
    private final SendableChooser<Command> autoChooser;
    // Commands requested by PathPlanner events that must run after the path finishes
    private final List<Supplier<Command>> m_postPathCommandSuppliers = new ArrayList<>();

    public RobotContainer() {
        configureNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        SmartDashboard.putNumber("Align/TriggerSeconds", 0.25);
        // Telemetry only paints hub virtual pose on /Pose when this is true (see Telemetry)
        SmartDashboard.putBoolean("Pose/PublishHubVirtualPose", false);

        configureBindings();
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureNamedCommands() {
        // INTAKE
        NamedCommands.registerCommand("Intake", intake.runOnce(()-> intake.setGoal(IntakeState.INTAKE)));
        NamedCommands.registerCommand("Stop Roller", intake.runOnce(()-> intake.setGoal(IntakeState.STOP)));
        NamedCommands.registerCommand("Agitate", intake.runOnce(()-> intake.setGoal(IntakeState.AGITATE)));
        NamedCommands.registerCommand("Stow", intake.runOnce(()-> intake.setGoal(IntakeState.STOW)));
        // SHOOTER
        NamedCommands.registerCommand("Hub Shot", shooter.runOnce(()-> shooter.setGoal(ShooterState.HUB)));
        NamedCommands.registerCommand("Idle Shot", shooter.runOnce(()-> shooter.setGoal(ShooterState.IDLE)));
        NamedCommands.registerCommand("Stop Shot", shooter.runOnce(()-> shooter.setGoal(ShooterState.STOP)));
        // HOOD
        NamedCommands.registerCommand("Hubbed Hood", hood.runOnce(()-> hood.setGoal(HoodState.HUB)));
        NamedCommands.registerCommand("Trenched Hood", hood.runOnce(()-> hood.setGoal(HoodState.TRENCH)));
        // INDEXER
        NamedCommands.registerCommand("Index", indexer.runOnce(() -> indexer.setGoal(IndexerState.INDEX)));
        NamedCommands.registerCommand("Stop Index", indexer.runOnce(() -> indexer.setGoal(IndexerState.STOP)));
        // KICKER
        NamedCommands.registerCommand("Kick", kicker.runOnce(() -> kicker.setGoal(KickerState.KICK)));
        NamedCommands.registerCommand(
            "Kick Until Resistance", kicker.runOnce(() -> kicker.setGoal(KickerState.KICK_UNTIL_RESISTANCE)));
        NamedCommands.registerCommand("Stop Kick", kicker.runOnce(() -> kicker.setGoal(KickerState.STOP)));
        // PP event can't require drive — stash align for after the path
        NamedCommands.registerCommand("Align Hub", Commands.runOnce(() -> {
            Supplier<Command> supplier = () -> autoAim.prepHubShot();
            double pulse = SmartDashboard.getNumber("Align/TriggerSeconds", 0.25);
            Command marker = Commands.sequence(
                Commands.runOnce(() -> SmartDashboard.putBoolean("Align/EventActive", true)),
                Commands.waitSeconds(pulse),
                Commands.runOnce(() -> SmartDashboard.putBoolean("Align/EventActive", false))
            );
            try {
                CommandScheduler.getInstance().schedule(marker);
            } catch (Exception ignored) {}
            m_postPathCommandSuppliers.add(supplier);
        }));
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
                () -> drive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate))
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        // HUB SHOT (hold RT = hub align — Telemetry uses this for Elastic hub pose)
        driver.rightTrigger().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Pose/PublishHubVirtualPose", true)));
        driver.rightTrigger().onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Pose/PublishHubVirtualPose", false)));
        driver.rightTrigger().onTrue(kicker.runOnce(() -> kicker.stopKickUntilResistance()));
        driver.rightTrigger().whileTrue(autoAim.prepHubShot());
        driver.rightTrigger().onTrue(shooter.runOnce(() -> shooter.setGoal(ShooterState.HUB)));
        driver.rightTrigger().onFalse(shooter.runOnce(() -> shooter.setGoal(ShooterState.IDLE)));
        driver.rightTrigger().onTrue(hood.runOnce(() -> hood.setGoal(HoodState.HUB)));
        driver.rightTrigger().onFalse(hood.runOnce(() -> hood.setGoal(HoodState.TRENCH)));

        // LEFT FERRY SHOT
        driver.povLeft().onTrue(kicker.runOnce(() -> kicker.stopKickUntilResistance()));
        driver.povLeft().whileTrue(autoAim.prepLeftFerryShot());
        driver.povLeft().onTrue(shooter.runOnce(() -> shooter.setGoal(ShooterState.LEFT_FERRY)));
        driver.povLeft().onFalse(shooter.runOnce(() -> shooter.setGoal(ShooterState.IDLE)));
        driver.povLeft().onTrue(hood.runOnce(() -> hood.setGoal(HoodState.FERRY)));
        driver.povLeft().onFalse(hood.runOnce(() -> hood.setGoal(HoodState.TRENCH)));

        // RIGHT FERRY SHOT
        driver.povRight().onTrue(kicker.runOnce(() -> kicker.stopKickUntilResistance()));
        driver.povRight().whileTrue(autoAim.prepRightFerryShot());
        driver.povRight().onTrue(shooter.runOnce(() -> shooter.setGoal(ShooterState.RIGHT_FERRY)));
        driver.povRight().onFalse(shooter.runOnce(() -> shooter.setGoal(ShooterState.IDLE)));
        driver.povRight().onTrue(hood.runOnce(() -> hood.setGoal(HoodState.FERRY)));
        driver.povRight().onFalse(hood.runOnce(() -> hood.setGoal(HoodState.TRENCH)));

        // INDEX TO SHOOTER (overrides kick-until-resistance if you mash Y)
        driver.y().onTrue(kicker.runOnce(() -> kicker.setGoal(KickerState.KICK)));
        driver.y().onFalse(kicker.runOnce(() -> kicker.setGoal(KickerState.STOP)));
        driver.y().onTrue(indexer.runOnce(() -> indexer.setGoal(IndexerState.INDEX)));
        driver.y().onFalse(indexer.runOnce(() -> indexer.setGoal(IndexerState.STOP)));
        driver.y().onTrue(intake.runOnce(() -> intake.setGoal(IntakeState.AGITATE)));
        driver.y().onFalse(intake.runOnce(() -> intake.setGoal(IntakeState.STOP)));

        // INTAKE (+ kick until resistance unless already aiming hub / ferry)
        driver.leftTrigger().onTrue(Commands.runOnce(() -> {
            intake.setGoal(IntakeState.INTAKE);
            if (!isDriverShootingInput()) {
                kicker.setGoal(KickerState.KICK_UNTIL_RESISTANCE);
            }
        }));
        driver.leftTrigger().onFalse(Commands.runOnce(() -> {
            intake.setGoal(IntakeState.STOP);
            kicker.stopKickUntilResistance();
        }));

        // OUTTAKE
        driver.b().onTrue(shooter.runOnce(() -> shooter.setGoal(ShooterState.OUTSHOOT)));
        driver.b().onFalse(shooter.runOnce(() -> shooter.setGoal(ShooterState.IDLE)));
        driver.b().onTrue(intake.runOnce(() -> intake.setGoal(IntakeState.OUTTAKE)));
        driver.b().onFalse(intake.runOnce(() -> intake.setGoal(IntakeState.STOP)));
        driver.b().onTrue(indexer.runOnce(() -> indexer.setGoal(IndexerState.OUTDEX)));
        driver.b().onFalse(indexer.runOnce(() -> indexer.setGoal(IndexerState.STOP)));
        driver.b().onTrue(kicker.runOnce(() -> kicker.setGoal(KickerState.OUTKICK)));
        driver.b().onFalse(kicker.runOnce(() -> kicker.setGoal(KickerState.STOP)));

        // STOW INTAKE
        driver.a().onTrue(intake.runOnce(() -> intake.setGoal(IntakeState.STOW)));
        driver.a().onFalse(intake.runOnce(() -> intake.setGoal(IntakeState.STOP)));

        // ZERO
        driver.start().onTrue(Commands.runOnce(() ->
            drivetrain.resetPose(new Pose2d(drivetrain.getState().Pose.getTranslation(), Rotation2d.kZero))));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        Command base = autoChooser.getSelected();
        if (base == null) {
            return null;
        }
        try {
            return Commands.sequence(
                base,
                Commands.runOnce(() -> {
                    for (var supplier : m_postPathCommandSuppliers) {
                        Command cmd = supplier.get();
                        if (cmd != null) {
                            CommandScheduler.getInstance().schedule(cmd);
                        }
                    }
                    m_postPathCommandSuppliers.clear();
                })
            );
        } catch (Exception ex) {
            System.err.println("Failed to compose autonomous sequence: " + ex);
            ex.printStackTrace();
            Logger.recordOutput("Autonomous/ComposeError", ex.toString());
            return base;
        }
    }

    /** True if driver is using hub or ferry shot — don’t start kick-until-resistance while intaking. */
    private boolean isDriverShootingInput() {
        return driver.getRightTriggerAxis() > 0.2
            || driver.povLeft().getAsBoolean()
            || driver.povRight().getAsBoolean();
    }
}
