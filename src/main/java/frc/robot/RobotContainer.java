// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.DoNothingAuto;
import frc.robot.autos.ConeCrossBalanceAuto;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final CommandXboxController  driverB = new CommandXboxController (0);
  private final CommandXboxController  operator = new CommandXboxController (1);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  /* Driver Buttons */
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value); //change in future
  private final JoystickButton slowSpeed =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value); //change in future

 /*Auto Selector */
 private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean(),
            () -> slowSpeed.getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();
    configureAutoSelector();

  }
  private void configureAutoSelector() {

    autoChooser.setDefaultOption("Cone Cross Balance Auto", new ConeCrossBalanceAuto(s_Swerve, intakeSubsystem, elevatorSubsystem, extensionSubsystem));
    autoChooser.addOption("Do Nothing Auto", new DoNothingAuto());
    autoChooser.addOption("Drive Forward Auto", new DriveForwardAuto(s_Swerve));
    SmartDashboard.putData("Auto Selector", autoChooser);

    // ShuffleboardTab tab = Shuffleboard.getTab("Auto Chooser");
    // Topic autoChooserEntry = tab.add("AutoChooser", 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry().getTopic();
    // autoChooserEntry.genericPublish("double").setDouble(autoChooser);

    

  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
     
/* Driver Buttons.......................................................................................................... */
    driverB.start().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));



   
    /*Intaking GamePiece */
     driverB.leftTrigger()
      .whileTrue(new SequentialCommandGroup(new InstantCommand(() -> { 
        intakeSubsystem.forwardIntakeMotor(); 
        extensionSubsystem.intakeExtension();
        }),
        new WaitCommand(0.5), 
        new InstantCommand(() -> 
        elevatorSubsystem.intakeElevator()
        )))
      .whileFalse(new SequentialCommandGroup(new InstantCommand(() -> {
        intakeSubsystem.stopIntakeMotor();
        elevatorSubsystem.stowedElevator();
        }),
        new WaitCommand(1.0),
        new InstantCommand(() ->
        extensionSubsystem.stowedExtension()
        )));

  
    /*SCORE */ 
      driverB.rightTrigger()
       .onTrue(new SequentialCommandGroup(new InstantCommand(() -> 
        intakeSubsystem.reverseIntakeMotor()),
        new WaitCommand(1.0),
        new InstantCommand(() -> {                    
        intakeSubsystem.stopIntakeMotor(); 
        extensionSubsystem.stowedExtension();
        elevatorSubsystem.stowedElevator();
        })));

//manual test of intake motor...remove when issue solved
          driverB.a().whileTrue(new StartEndCommand(() -> intakeSubsystem.reverseIntakeMotor(),
          () -> intakeSubsystem.stopIntakeMotor() 
          ));


/* Operator Buttons......................................................................................................... */
    
    /*Positions Elevator/Extension*/
    operator.a().onTrue(new InstantCommand(() -> {elevatorSubsystem.l2Elevator(); extensionSubsystem.l2Extension(); }));
    operator.y().onTrue(new InstantCommand(() -> {elevatorSubsystem.l3Elevator(); extensionSubsystem.l3Extension(); }));
   
    /*Positions Human Station*/
    operator.b().whileTrue(new InstantCommand(() -> {
        extensionSubsystem.humanExtension();
        elevatorSubsystem.humanElevator();
        intakeSubsystem.forwardIntakeMotor();
          }));
    operator.b().whileFalse(new InstantCommand(() -> {
        extensionSubsystem.stowedExtension();
        elevatorSubsystem.stowedElevator();
        intakeSubsystem.stopIntakeMotor();
          }));

    /*Manual Elevator and Extension */
    operator.povDown().whileTrue(new StartEndCommand(() -> elevatorSubsystem.downElevatorMotor(),
      () -> elevatorSubsystem.stopElevatorMotor() ));
    operator.povUp().whileTrue(new StartEndCommand(() -> elevatorSubsystem.upElevatorMotor(),
      () -> elevatorSubsystem.stopElevatorMotor()) );
    operator.povLeft().whileTrue(new StartEndCommand(() -> extensionSubsystem.forwardExtensionMotor(),
      () -> extensionSubsystem.stopExtensionMotor() ));
    operator.povRight().whileTrue(new StartEndCommand(() -> extensionSubsystem.reverseExtensionMotor(),
      () -> extensionSubsystem.stopExtensionMotor() ));



  }  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}