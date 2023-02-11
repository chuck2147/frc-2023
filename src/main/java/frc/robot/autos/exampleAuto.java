package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class exampleAuto extends ParallelCommandGroup {
 
  public exampleAuto(Swerve s_Swerve, IntakeSubsystem intakeSubsystem) {
    addRequirements(s_Swerve, intakeSubsystem);
    
    addCommands(
      new RunCommand(() -> s_Swerve.drive(new Translation2d(0.3, 0), 0, true, true )).withTimeout(3),
      new RunCommand(() -> s_Swerve.drive(new Translation2d(-0.3, 0), 0, true, true )).withTimeout(3)
    );
    
    addCommands(
      new RunCommand(() -> intakeSubsystem.forwardIntakeMotor()).withTimeout(3),
      new RunCommand(() -> intakeSubsystem.stopIntakeMotor()).withTimeout(3)//,
      // new InstantCommand(()->s_Swerve.resetOdometry(null))
    );
    
  }
}
