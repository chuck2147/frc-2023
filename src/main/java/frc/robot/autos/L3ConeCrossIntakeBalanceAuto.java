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

public class L3ConeCrossIntakeBalanceAuto extends SequentialCommandGroup {

    public L3ConeCrossIntakeBalanceAuto(Swerve s_Swerve, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem,
            ExtensionSubsystem extensionSubsystem) {
        addRequirements(s_Swerve, intakeSubsystem, elevatorSubsystem, extensionSubsystem);

        addCommands(
                new InstantCommand(() -> s_Swerve.setGyro(180)), 
                Commands.parallel(
                        new RunCommand(() -> {
                            elevatorSubsystem.l3Elevator();
                        }).withTimeout(1.5),
                        new WaitCommand(0.6).andThen(new RunCommand(() -> {
                            extensionSubsystem.l3Extension();
                        }).withTimeout(0.7))),
                new RunCommand(() -> intakeSubsystem.reverseIntakeMotor()).withTimeout(0.5),
                new InstantCommand(() -> {
                    intakeSubsystem.stopIntakeMotor();
                    elevatorSubsystem.stowedElevator();
                    extensionSubsystem.stowedExtension();
                }),
                new RunCommand(() -> s_Swerve.drive(new Translation2d(1.7, 0), 0, true, true)).withTimeout(2),

                new RunCommand(() -> s_Swerve.drive(new Translation2d(1.7, 0), 0, true, true)).withTimeout(0.5),
                new RunCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 4.70, true, true)).withTimeout(.7),
                new RunCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)).withTimeout(0.07),
               
                
                    
                new RunCommand(() -> {
                    extensionSubsystem.intakeExtension();
                    intakeSubsystem.forwardIntakeMotor();
                }).withTimeout(0.37),
                new WaitCommand(0.37).andThen(new RunCommand(() -> {
                    elevatorSubsystem.intakeElevator();
                }).withTimeout(0.7)),
                    

                new RunCommand(() -> s_Swerve.drive(new Translation2d(1.3, 0), 0, true, true)).withTimeout(.47),
                new WaitCommand(0.9).andThen(
                new RunCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)).withTimeout(0.07)),
                    //one more WaitCommand

                
                    
                new RunCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)).withTimeout(0.3),

              
            
                Commands.parallel(

                    new RunCommand(() -> s_Swerve.drive(new Translation2d(-1.3, 0), 0, true, true)).withTimeout(.47),
                    new RunCommand(() -> s_Swerve.drive(new Translation2d(-1.7, 0), 0, true, true)).withTimeout(2.157),

                    /*new WaitCommand(2.157).andThen(
                    new RunCommand(() -> s_Swerve.drive(new Translation2d(-1.7, 0), 0, true, true))).withTimeout(0.6471),
            */

                
                    new RunCommand(() -> {
                        elevatorSubsystem.stowedElevator();
                    }).withTimeout(0.5),
                    new WaitCommand(0.5).andThen(new RunCommand(() -> {
                        extensionSubsystem.stowedExtension();
                        intakeSubsystem.stopIntakeMotor();
                        s_Swerve.drive(new Translation2d(-1.7, 0), 0, true, true);
                    }).withTimeout(0.50329999))),
                
                    
                new BalanceCommand(s_Swerve)
        );
        // addCommands(
        // new RunCommand(() -> intakeSubsystem.forwardIntakeMotor()).withTimeout(3),
        // new RunCommand(() -> intakeSubsystem.stopIntakeMotor()).withTimeout(3)//,
        // // new InstantCommand(()->s_Swerve.resetOdometry(null))
        // );

    }
}
