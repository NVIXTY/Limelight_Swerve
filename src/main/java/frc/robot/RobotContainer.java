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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Climb.Climb;
<<<<<<< HEAD
import frc.robot.subsystems.Drive.DriveConstants;
import frc.robot.subsystems.Drive.DrivetrainFactory;
import frc.robot.subsystems.Drive.SwerveDrivetrain;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Index.Index;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.commands.PrepHubShot;
import frc.robot.commands.PrepLeftFerryShot;
import frc.robot.commands.PrepRightFerryShot;
=======
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.DriveConstants;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Index.Index;
import frc.robot.subsystems.Intake.Intake;
>>>>>>> 995b5c5 (cleaned up robotcontainer, added support for dual motor on shooter.)

public class RobotContainer {
    private final double MaxSpeed = 5.12 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

    // Drive requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
<<<<<<< HEAD
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // removed unused requests to silence linter warnings
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // robotCentricDrive unused
=======
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(DriveConstants.maxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
>>>>>>> 995b5c5 (cleaned up robotcontainer, added support for dual motor on shooter.)

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

<<<<<<< HEAD

    public final SwerveDrivetrain drivetrain = frc.robot.subsystems.Drive.DrivetrainFactory.create();

    // (No coordinator) Use direct command composition for higher-level actions
    // Direct subsystem instance
=======
    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
>>>>>>> 995b5c5 (cleaned up robotcontainer, added support for dual motor on shooter.)
    private final Intake slapdownIntake = new Intake();
    private final Index index = new Index();
    private final Climb climb = new Climb(50); // TODO: move to ClimbConstants.MOTOR_ID if desired

<<<<<<< HEAD
    // Toggle state (starts stowed)
    private boolean intakeDeployed = false;
    private boolean climbDeployed = false;
    /* Path follower */
=======
    // States
    private boolean intakeDeployed = false;
    private boolean climbDeployed = false;

    // Auto
>>>>>>> 995b5c5 (cleaned up robotcontainer, added support for dual motor on shooter.)
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
<<<<<<< HEAD
    // Warmup PathPlanner to avoid Java pauses
    // `Command.schedule()` is deprecated; use the CommandScheduler to schedule commands.
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
=======
        FollowPathCommand.warmupCommand().schedule();
>>>>>>> 995b5c5 (cleaned up robotcontainer, added support for dual motor on shooter.)
    }

    private void configureBindings() {
        configureDriveBindings();
        configureIntakeAndIndexBindings();
        configureClimbBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureDriveBindings() {
        drivetrain.setDefaultCommand(
<<<<<<< HEAD
            // Drivetrain will execute this command periodically. Use Commands.run to explicitly attach the drivetrain requirement.
            Commands.run(() -> {
                drivetrain.setControl(
                    drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                );
            }, drivetrain)
=======
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate))
>>>>>>> 995b5c5 (cleaned up robotcontainer, added support for dual motor on shooter.)
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            Commands.run(() -> drivetrain.setControl(idle), drivetrain).ignoringDisable(true)
        );

<<<<<<< HEAD
    driver.x().whileTrue(new PrepHubShot(drivetrain, driver));
    driver.povLeft().whileTrue(new PrepLeftFerryShot(drivetrain, driver));
    driver.povRight().whileTrue(new PrepRightFerryShot(drivetrain, driver));

        driver.povUp().whileTrue(
            Commands.run(() -> drivetrain.setControl(forwardStraight.withVelocityX(0.5).withVelocityY(0)), drivetrain)
        );
        driver.povDown().whileTrue(
            Commands.run(() -> drivetrain.setControl(forwardStraight.withVelocityX(-0.5).withVelocityY(0)), drivetrain)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
=======
>>>>>>> 995b5c5 (cleaned up robotcontainer, added support for dual motor on shooter.)
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

<<<<<<< HEAD
        // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(Commands.runOnce(drivetrain::seedFieldCentric, drivetrain));
=======
        driver.rightBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }
>>>>>>> 995b5c5 (cleaned up robotcontainer, added support for dual motor on shooter.)

    private void configureIntakeAndIndexBindings() {
        // Intake pivot manual (operator)
        operator.a()
            .whileTrue(Commands.run(() -> slapdownIntake.manualControl(0.1), slapdownIntake))
            .onFalse(Commands.runOnce(slapdownIntake::stop, slapdownIntake));

        operator.x()
<<<<<<< HEAD
        .whileTrue(Commands.run(() -> slapdownIntake.manualControl(-0.2), slapdownIntake))
        .onFalse(Commands.runOnce(slapdownIntake::stop, slapdownIntake));
        
        
        // B: toggle deploy/retract on each press
=======
            .whileTrue(Commands.run(() -> slapdownIntake.manualControl(-0.1), slapdownIntake))
            .onFalse(Commands.runOnce(slapdownIntake::stop, slapdownIntake));

        // Intake deploy/retract toggle (driver B)
>>>>>>> 995b5c5 (cleaned up robotcontainer, added support for dual motor on shooter.)
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

    private void configureIntakeAndIndexBindings() {
        // Intake pivot manual (operator)
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
