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
import frc.robot.commands.BalanceCommand;
import frc.robot.subsystems.Swerve;

public class L2ConeCrossIntakeBalanceAuto extends SequentialCommandGroup {

    public L2ConeCrossIntakeBalanceAuto(Swerve s_Swerve, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem,
            ExtensionSubsystem extensionSubsystem) {
        addRequirements(s_Swerve, intakeSubsystem, elevatorSubsystem, extensionSubsystem);

        addCommands(
                new InstantCommand(() -> s_Swerve.setGyro(180)), //180
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
                new RunCommand(() -> s_Swerve.drive(new Translation2d(1.7, 0), 0, true, true)).withTimeout(2),

                new RunCommand(() -> s_Swerve.drive(new Translation2d(1.7, 0), 180, true, true)).withTimeout(1.2),

                Commands.parallel(
                    new RunCommand(() -> {
                        extensionSubsystem.intakeExtension();
                    }).withTimeout(0.5),
                    new WaitCommand(0.5).andThen(new RunCommand(() -> {
                        elevatorSubsystem.intakeElevator();
                    }).withTimeout(1.5))),
              
                new RunCommand(() -> intakeSubsystem.forwardIntakeMotor()).withTimeout(0.5), //change
            
                Commands.parallel(
                    new RunCommand(() -> {
                        elevatorSubsystem.stowedElevator();
                        intakeSubsystem.reverseIntakeMotor();
                    }).withTimeout(1),
                    new WaitCommand(0.5).andThen(new RunCommand(() -> {
                        extensionSubsystem.stowedExtension();
                        s_Swerve.drive(new Translation2d(-1.7, 0), 180, true, true);
                    }).withTimeout(1.5099))),
    
                new RunCommand(() -> s_Swerve.drive(new Translation2d(-1.7, 0), 0, true, true)).withTimeout(0.6471),
                
                new BalanceCommand(s_Swerve)
        );
        // addCommands(
        // new RunCommand(() -> intakeSubsystem.forwardIntakeMotor()).withTimeout(3),
        // new RunCommand(() -> intakeSubsystem.stopIntakeMotor()).withTimeout(3)//,
        // // new InstantCommand(()->s_Swerve.resetOdometry(null))
        // );

    }
}
