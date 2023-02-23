package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class exampleAuto extends SequentialCommandGroup {
 
  public exampleAuto(Swerve s_Swerve, IntakeSubsystem intakeSubsystem) {
    addRequirements(s_Swerve, intakeSubsystem);
  
    addCommands(
      new RunCommand(() -> s_Swerve.drive(new Translation2d(1.7, 0), 0, true, true )).withTimeout(1.7),
      new RunCommand(() -> s_Swerve.drive(new Translation2d(2.4, 0), 0, true, true )).withTimeout(1.25),
      new RunCommand(() -> s_Swerve.drive(new Translation2d(-1.7, 0), 0, true, true )).withTimeout(4.7)
    );
    
    // addCommands(
    //   new RunCommand(() -> intakeSubsystem.forwardIntakeMotor()).withTimeout(3),
    //   new RunCommand(() -> intakeSubsystem.stopIntakeMotor()).withTimeout(3)//,
    //   // new InstantCommand(()->s_Swerve.resetOdometry(null))
    // );
    
  }
}
