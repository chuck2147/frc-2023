package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Swerve;

public class DriveForwardAuto extends ParallelCommandGroup {
 
  public DriveForwardAuto(Swerve s_Swerve) {
    addRequirements(s_Swerve);
    
    addCommands(
      new RunCommand(() -> s_Swerve.drive(new Translation2d(0.7, 0), 0, true, true )).withTimeout(2.2)
    );
    
    // addCommands(
    //   new RunCommand(() -> intakeSubsystem.forwardIntakeMotor()).withTimeout(3),
    //   new RunCommand(() -> intakeSubsystem.stopIntakeMotor()).withTimeout(3)//,
    //   // new InstantCommand(()->s_Swerve.resetOdometry(null))
    // );
    
  }
}
