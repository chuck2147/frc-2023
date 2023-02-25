package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class BalanceCommand extends CommandBase {
  private Swerve s_Swerve;

  public BalanceCommand(
      Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {
    double translationVal = s_Swerve.getPitch().getDegrees() * -0.006;
    s_Swerve.drive(
        new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed),
        0,
        false,
        true);
  }
}
