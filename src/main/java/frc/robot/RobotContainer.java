// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // LIMELIGHT ALIGNMENT BINDINGS - D-pad left/right
        // D-pad LEFT (POV 270) = Left Lineup
        joystick.pov(270)
            .whileTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 0)),
                    drivetrain.applyRequest(() -> {
                        if (LimelightHelpers.getTV("limelight")) {
                            int tagId = (int) LimelightHelpers.getFiducialID("limelight");
                            double targetAngle = getReefAngle(tagId);
                            
                            SmartDashboard.putNumber("Limelight/Left/TagID", tagId);
                            SmartDashboard.putNumber("Limelight/Left/TargetAngle", targetAngle);
                            SmartDashboard.putNumber("Limelight/Left/TX", LimelightHelpers.getTX("limelight"));
                            
                            return robotCentricDrive
                                .withVelocityY(LimelightHelpers.getTX("limelight") * -limelightMaxSpeed)
                                .withRotationalRate(angLock(targetAngle));
                        } else {
                            SmartDashboard.putString("Limelight/Left/Status", "No Valid Target");
                            return robotCentricDrive.withVelocityY(0).withRotationalRate(0);
                        }
                    })
                )
            );

        // D-pad RIGHT (POV 90) = Right Lineup  
        joystick.pov(90)
            .whileTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 1)),
                    drivetrain.applyRequest(() -> {
                        if (LimelightHelpers.getTV("limelight")) {
                            int tagId = (int) LimelightHelpers.getFiducialID("limelight");
                            double targetAngle = getReefAngle(tagId);
                            
                            SmartDashboard.putNumber("Limelight/Right/TagID", tagId);
                            SmartDashboard.putNumber("Limelight/Right/TargetAngle", targetAngle);
                            SmartDashboard.putNumber("Limelight/Right/TX", LimelightHelpers.getTX("limelight"));
                            
                            return robotCentricDrive
                                .withVelocityY(LimelightHelpers.getTX("limelight") * -limelightMaxSpeed)
                                .withRotationalRate(angLock(targetAngle));
                        } else {
                            SmartDashboard.putString("Limelight/Right/Status", "No Valid Target");
                            return robotCentricDrive.withVelocityY(0).withRotationalRate(0);
                        }
                    })
                )
            );


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->forwardStraight
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(-0.5).withVelocityY(-.5))
    );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

// Add this field for Limelight speed control
private final double limelightMaxSpeed = 3.0;

    /**
     * Get the reef angle based on AprilTag ID
     */
    private double getReefAngle(int id) {
        switch (id) {
            case 6: return 120;
            case 7: return 180;
            case 8: return 240;
            case 9: return 300;
            case 10: return 0;    // 360 normalized to 0
            case 11: return 60;   // 420 normalized to 60
            case 17: return 60;
            case 18: return 0;
            case 19: return -60;
            case 20: return -120;
            case 21: return 180;
            case 22: return 120;
            default: return 180;
        }
    }

    /**
     * Register Limelight alignment commands for autonomous use
     */
    private void registerLimelightCommands() {
        // Left Lineup Command - Pipeline 0
        NamedCommands.registerCommand("Left Lineup", 
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 2)),
                drivetrain.applyRequest(() -> {
                    // Only align if Limelight has a valid target
                    if (LimelightHelpers.getTV("limelight")) {
                        // Get the AprilTag ID from Limelight and determine target angle
                        int tagId = (int) LimelightHelpers.getFiducialID("limelight");
                        double targetAngle = getReefAngle(tagId);
                        
                        // Debug output
                        SmartDashboard.putNumber("Limelight/Left/TagID", tagId);
                        SmartDashboard.putNumber("Limelight/Left/TargetAngle", targetAngle);
                        SmartDashboard.putNumber("Limelight/Left/TX", LimelightHelpers.getTX("limelight"));
                        
                        return robotCentricDrive
                            .withVelocityY(LimelightHelpers.getTX("limelight") * limelightMaxSpeed)
                            .withRotationalRate(angLock(targetAngle));
                    } else {
                        // No valid target, stop movement
                        SmartDashboard.putString("Limelight/Left/Status", "No Valid Target");
                        return robotCentricDrive.withVelocityY(0).withRotationalRate(0);
                    }
                }).withTimeout(2.5)
            )
        );
        
        // Right Lineup Command - Pipeline 1
        NamedCommands.registerCommand("Right Lineup", 
            new SequentialCommandGroup(
                new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 1)),
                drivetrain.applyRequest(() -> {
                    // Only align if Limelight has a valid target
                    if (LimelightHelpers.getTV("limelight")) {
                        // Get the AprilTag ID from Limelight and determine target angle
                        int tagId = (int) LimelightHelpers.getFiducialID("limelight");
                        double targetAngle = getReefAngle(tagId);
                        
                        // Debug output
                        SmartDashboard.putNumber("Limelight/Right/TagID", tagId);
                        SmartDashboard.putNumber("Limelight/Right/TargetAngle", targetAngle);
                        SmartDashboard.putNumber("Limelight/Right/TX", LimelightHelpers.getTX("limelight"));
                        
                        return robotCentricDrive
                            .withVelocityY(LimelightHelpers.getTX("limelight") * -limelightMaxSpeed)
                            .withRotationalRate(angLock(targetAngle));
                    } else {
                        // No valid target, stop movement
                        SmartDashboard.putString("Limelight/Right/Status", "No Valid Target");
                        return robotCentricDrive.withVelocityY(0).withRotationalRate(0);
                    }
                }).withTimeout(2.5)
            )
        );
    }

   /**
     * Angle lock function to maintain or rotate to a target angle
     * @param targetAngle Target angle in degrees
     * @return Angular velocity in radians per second
     */
    public double angLock(double targetAngle) {
        double currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();
        double targetDeg = targetAngle;
        double currentDeg = currentAngle;
        
        // Calculate the shortest angular distance
        double offset = targetDeg - currentDeg;
    
        // Normalize angle to [-180, 180] range
        while (offset > 180) {
            offset -= 360;
        }
        while (offset < -180) {
            offset += 360;
        }

        // Convert to radians and apply gain
        double angLock = (offset * Math.PI / 180.0 * 10);

        // Apply deadband
        if (Math.abs(offset) < 2) {
            angLock = 0;
        }
        
        // Debug output
        SmartDashboard.putNumber("AngLock/Offset", offset);
        SmartDashboard.putNumber("AngLock/Current", currentDeg);
        SmartDashboard.putNumber("AngLock/Target", targetDeg);
        SmartDashboard.putNumber("AngLock/Output", angLock);
        
        return angLock;
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
