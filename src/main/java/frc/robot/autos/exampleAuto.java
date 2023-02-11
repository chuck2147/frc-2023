package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class exampleAuto extends ParallelCommandGroup {
  public exampleAuto(Swerve s_Swerve) {
    addRequirements(s_Swerve);
    addCommands(
      new RunCommand(() -> s_Swerve.drive(new Translation2d(0.3, 0), 0, true, true )).withTimeout(3),
      new RunCommand(() -> s_Swerve.drive(new Translation2d(-0.3, 0), 0, true, true )).withTimeout(3)
    );
    // addCommands(
    //   new RunCommand(() -> IntakeSubsystem.
    // );
    //
  }
}
