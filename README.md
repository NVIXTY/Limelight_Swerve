# 2026 Robot Code (Steel Falcons)

This repository contains the 2026 robot code for team 7220 - Steel Falcons.

## Recent migration notes
- Replaced a team-specific `Superstructure` coordinator with WPILib-style commands and command composition.
  - The legacy `src/main/java/frc/robot/Superstructure.java` was removed (content replaced with a removal note).
  - Use these command classes instead:
    - `frc.robot.commands.PrepHubShot`
    - `frc.robot.commands.PrepLeftFerryShot`
    - `frc.robot.commands.PrepRightFerryShot`
- CTRE Phoenix-generated drivetrain code is kept intact. To avoid editing generated files, a small non-generated factory was added:
  - `frc.robot.subsystems.Drive.DrivetrainFactory.create()` constructs the application-visible `SwerveDrivetrain` wrapper.
  - Application code should use the `SwerveDrivetrain` wrapper (or the factory) rather than modifying generated types.

If you need help reverting the migration or moving back to a coordinator approach, ask and I can restore or adapt the code.
