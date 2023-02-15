package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class emptyAuto extends SequentialCommandGroup {
 
  public emptyAuto(Swerve s_Swerve, IntakeSubsystem intakeSubsystem) {
    addRequirements(s_Swerve, intakeSubsystem);
    
    // addCommands(
    //   new RunCommand(() -> s_Swerve.drive(new Translation2d(0.3, 0), 0, true, true )).withTimeout(3)
    //             );

      // new InstantCommand(()->s_Swerve.resetOdometry(null))
    
    
  }
}
