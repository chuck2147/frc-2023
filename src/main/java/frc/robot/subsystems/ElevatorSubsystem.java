// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDNTValue;

public class ElevatorSubsystem extends SubsystemBase {
private TalonFX elevatorMotor = new TalonFX(Constants.ELEVATOR_MOTOR_ID);
private TalonFX elevatorMotorFollower = new TalonFX(Constants.ELEVATOR_FOLLOWER_MOTOR_ID);
  
  private DoubleSolenoid elevatorBreak = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
  Constants.ELEVATOR_BRAKE_FORWARD, Constants.ELEVATOR_BRAKE_REVERSE);

  // PID coefficients............................................
  double kP = 0.01; 
  double kI = 0;
  double kD = 0; 
  double kF = 0; 

  //PID Setpoint....................................................
  double l3ElevatorPosition = 100000;
  double l2ElevatorPosition = 50000;
  double humanElevatorPosition = 25000;
  double stowedElevatorPosition = 0; //starting configuration set when robot turned on
  double intakeElevatorPosition = -10000; //negative because will be lower than starting configuration



  ShuffleboardTab tab = Shuffleboard.getTab("NTValues");
  
;
  Topic l2ElevatorPositionEntry = tab.add("L2 Elevator Position", 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry().getTopic();
  // Topic l2ElevatorPositionGraphEntry = tab.add("L2 Graph Elevator Position", 0).withSize(2, 1).withWidget(BuiltInWidgets.kGraph).getEntry().getTopic();
 
  Topic l3ElevatorPositionEntry = tab.add("L3 Elevator Position", 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry().getTopic();
  //Topic l3ElevatorPositionGraphEntry = tab.add("L3 Graph Elevator Position", 0).withSize(2, 1).withWidget(BuiltInWidgets.kGraph).getEntry().getTopic();
 
  Topic humanElevatorPositionEntry = tab.add("Human Elevator Position", 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry().getTopic();
  //Topic humanElevatorPositionGraphEntry = tab.add("Human Graph Elevator Position", 0).withSize(2, 1).withWidget(BuiltInWidgets.kGraph).getEntry().getTopic();
 
  Topic stowedElevatorPositonEntry = tab.add("Stowed Elevator Position", 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry().getTopic();
  //Topic stowedElevatorPositonGraphEntry = tab.add("Stowed Graph Elevator Position", 0).withSize(2, 1).withWidget(BuiltInWidgets.kGraph).getEntry().getTopic();
 
  Topic intakeElevatorPositionEntry = tab.add("Intake Elevator Position", 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry().getTopic();
  // Topic intakeElevatorPositionGraphEntry = tab.add("Intake Graph Elevator Position", 0).withSize(2, 1).withWidget(BuiltInWidgets.kGraph).getEntry().getTopic();
  

  
  public ElevatorSubsystem() {

    elevatorMotor.configFactoryDefault();
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    elevatorMotor.setInverted(TalonFXInvertType.Clockwise);//check
    elevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);  
    elevatorMotor.setSensorPhase(true);//check
  
    /* set up followers */
    elevatorMotorFollower.configFactoryDefault();
    elevatorMotorFollower.setNeutralMode(NeutralMode.Brake);
    elevatorMotorFollower.setInverted(TalonFXInvertType.CounterClockwise);//check
    elevatorMotorFollower.follow(elevatorMotor);

    // set PID coefficients
    elevatorMotor.config_kP(0, kP, 30); 
    elevatorMotor.config_kI(0, kI, 30); 
    elevatorMotor.config_kD(0, kD, 30); 
    elevatorMotor.config_kF(0, kF, 30); 

    new PIDNTValue(kP, kI, kD, kF, elevatorMotor,"Elevator Motor");
  }
 
  

  public void l3Elevator() {
    elevatorMotor.set(ControlMode.Position, l3ElevatorPosition);
    elevatorBreak.set(Value.kForward);
  }

  public void l2Elevator() {
    elevatorMotor.set(ControlMode.Position, l2ElevatorPosition);
    elevatorBreak.set(Value.kForward);
  }

  public void stowedElevator() {
    elevatorMotor.set(ControlMode.Position, stowedElevatorPosition);
    elevatorBreak.set(Value.kForward);
  }

  public void intakeElevator() {
    elevatorMotor.set(ControlMode.Position, intakeElevatorPosition);
    elevatorBreak.set(Value.kForward);
  }
  
  public void humanElevator() {
    elevatorMotor.set(ControlMode.Position, humanElevatorPosition);
    elevatorBreak.set(Value.kForward);
  }

    public void upElevatorMotor() {
      System.out.println("going up!");
      elevatorMotor.set(ControlMode.PercentOutput, 1);
      elevatorBreak.set(Value.kForward);
    }
  
    public void downElevatorMotor() {
      System.out.println("going down...");
      elevatorMotor.set(ControlMode.PercentOutput, -0.5);
      elevatorBreak.set(Value.kForward);
    }
  
    public void stopElevatorMotor() {
      elevatorMotor.set(ControlMode.PercentOutput, 0);
      elevatorBreak.set(Value.kReverse);
  }

    public double getElevatorEncoder() {
      return elevatorMotor.getSelectedSensorVelocity();
    }  
  
  //Do we need this?
    public void resetElevatorEncoder() {
      elevatorMotor.set(ControlMode.Position, stowedElevatorPosition);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Elevator Encoder", getElevatorEncoder());

    l2ElevatorPositionEntry.genericPublish("double").setDouble(l2ElevatorPosition);
    // l2ElevatorPositionGraphEntry.genericPublish("double").setDouble(l2ElevatorPosition);

    l3ElevatorPositionEntry.genericPublish("double").setDouble(l3ElevatorPosition);
    // l3ElevatorPositionGraphEntry.genericPublish("double").setDouble(l3ElevatorPosition);

     humanElevatorPositionEntry.genericPublish("double").setDouble(humanElevatorPosition);
    // humanElevatorPositionGraphEntry.genericPublish("double").setDouble(humanElevatorPosition);

    stowedElevatorPositonEntry.genericPublish("double").setDouble(stowedElevatorPosition);
    // stowedElevatorPositonGraphEntry.genericPublish("double").setDouble(stowedElevatorPosition);

    intakeElevatorPositionEntry.genericPublish("double").setDouble(intakeElevatorPosition);
    // intakeElevatorPositionGraphEntry.genericPublish("double").setDouble(intakeElevatorPosition);

    
  }

  @Override
  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation
  }
}
