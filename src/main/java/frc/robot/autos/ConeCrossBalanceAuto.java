package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class ConeCrossBalanceAuto extends SequentialCommandGroup {

    public ConeCrossBalanceAuto(Swerve s_Swerve, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem,
            ExtensionSubsystem extensionSubsystem) {
        addRequirements(s_Swerve, intakeSubsystem, elevatorSubsystem, extensionSubsystem);

        addCommands(
                new InstantCommand(() -> s_Swerve.setGyro(180)),
                Commands.parallel(
                        new RunCommand(() -> {
                            elevatorSubsystem.l2Elevator();
                        }).withTimeout(1.5),
                        new WaitCommand(0.6).andThen(new RunCommand(() -> {
                            extensionSubsystem.l2Extension();
                        }).withTimeout(1.5))),
                new RunCommand(() -> intakeSubsystem.reverseIntakeMotor()).withTimeout(0.5),
                new InstantCommand(() -> {
                    intakeSubsystem.stopIntakeMotor();
                    elevatorSubsystem.stowedElevator();
                    extensionSubsystem.stowedExtension();
                }),
                new RunCommand(() -> s_Swerve.drive(new Translation2d(1.7, 0), 0, true, true)).withTimeout(3.2),
                new RunCommand(() -> s_Swerve.drive(new Translation2d(-1.7, 0), 0, true, true)).withTimeout(2.157)
                    );
        // addCommands(
        // new RunCommand(() -> intakeSubsystem.forwardIntakeMotor()).withTimeout(3),
        // new RunCommand(() -> intakeSubsystem.stopIntakeMotor()).withTimeout(3)//,
        // // new InstantCommand(()->s_Swerve.resetOdometry(null))
        // );

    }
}
