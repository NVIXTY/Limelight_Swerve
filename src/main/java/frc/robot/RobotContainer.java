// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.DriveConstants;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Index.Index;
import frc.robot.subsystems.Intake.Intake;

public class RobotContainer {
    private final double MaxSpeed = 5.12 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

    // Drive requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(DriveConstants.maxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake slapdownIntake = new Intake();
    private final Index index = new Index();
    private final Climb climb = new Climb(50); // TODO: move to ClimbConstants.MOTOR_ID if desired

    // States
    private boolean intakeDeployed = false;
    private boolean climbDeployed = false;

    // Auto
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        configureDriveBindings();
        configureIntakeAndIndexBindings();
        configureClimbBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureDriveBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate))
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driver.rightBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    private void configureIntakeAndIndexBindings() {
        // Intake pivot manual (operator)
        operator.a()
            .whileTrue(Commands.run(() -> slapdownIntake.manualControl(0.1), slapdownIntake))
            .onFalse(Commands.runOnce(slapdownIntake::stop, slapdownIntake));

        operator.x()
            .whileTrue(Commands.run(() -> slapdownIntake.manualControl(-0.1), slapdownIntake))
            .onFalse(Commands.runOnce(slapdownIntake::stop, slapdownIntake));

        // Intake deploy/retract toggle (driver B)
        driver.b().onTrue(
            Commands.runOnce(() -> {
                if (intakeDeployed) {
                    slapdownIntake.retract();
                } else {
                    slapdownIntake.deploy();
                }
                intakeDeployed = !intakeDeployed;
            }, slapdownIntake)
        );

        // Index forward while held (driver Y)
        driver.y()
            .whileTrue(Commands.run(index::runIndex, index))
            .onFalse(Commands.runOnce(index::stop, index));

        // OUTTAKE while held (driver left bumper):
        //  - roller reverse
        //  - index reverse
        driver.leftBumper()
            .whileTrue(Commands.run(() -> {
                slapdownIntake.runRollerReverse();
                index.runIndexReverse();
            }, slapdownIntake, index))
            .onFalse(Commands.runOnce(() -> {
                slapdownIntake.stopRoller();
                index.stop();
            }, slapdownIntake, index));
    }

    private void configureClimbBindings() {
        // Manual climb control (operator): Y up, A down
        operator.y()
            .whileTrue(Commands.run(() -> climb.manualControl(0.2), climb))
            .onFalse(Commands.runOnce(climb::stop, climb));

        operator.a()
            .whileTrue(Commands.run(() -> climb.manualControl(-0.2), climb))
            .onFalse(Commands.runOnce(climb::stop, climb));

        // Toggle climb setpoint (operator B)
        operator.b().onTrue(
            Commands.runOnce(() -> {
                if (climbDeployed) {
                    climb.retract();
                } else {
                    climb.deploy();
                }
                climbDeployed = !climbDeployed;
            }, climb)
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
