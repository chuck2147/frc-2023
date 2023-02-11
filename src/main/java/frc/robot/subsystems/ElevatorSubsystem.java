// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkMax elevatorMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder elevator_encoder;
  private SparkMaxPIDController elevator_pidController;  
  
  private DoubleSolenoid elevatorBreak = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
  Constants.ELEVATOR_BRAKE_FORWARD, Constants.ELEVATOR_BRAKE_REVERSE);

// PID coefficients............................................
double kP = 0.01; 
double kI = 0;
double kD = 0; 
double kIz = 0; 
double kFF = 0; 
double kMaxOutput = 1; 
double kMinOutput = -1;

//PID Setpoint....................................................
double l3ElevatorPosition = 100;
double l2ElevatorPosition = 50;
double stowedElevatorPosition = 0;

  public ElevatorSubsystem() {
    elevatorMotor.restoreFactoryDefaults();
    elevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    elevatorMotor.setInverted(false); 
  
    elevatorMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    elevatorMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
  
    elevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15); //what is 15?
    elevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
  
    elevator_encoder = elevatorMotor.getEncoder();
         
    elevator_pidController = elevatorMotor.getPIDController();
    
    // set PID coefficients
    elevator_pidController.setP(kP); 
    elevator_pidController.setI(kI);
    elevator_pidController.setD(kD);
    elevator_pidController.setIZone(kIz);
    elevator_pidController.setFF(kFF);
    elevator_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void l3Elevator() {
    elevator_pidController.setReference(l3ElevatorPosition, CANSparkMax.ControlType.kPosition);
    elevatorBreak.set(Value.kForward);
  }

  public void l2Elevator() {
    elevator_pidController.setReference(l2ElevatorPosition, CANSparkMax.ControlType.kPosition);
    elevatorBreak.set(Value.kForward);
  }

  public void stowedElevator() {
    elevator_pidController.setReference(stowedElevatorPosition, CANSparkMax.ControlType.kPosition);
    elevatorBreak.set(Value.kForward);
  }

  public void breakElevator() {
    elevatorBreak.set(Value.kReverse);
  }

  public void resetEncoder() {
    elevator_encoder.setPosition(0);
    }

    public void upElevatorMotor() {
      System.out.println("going up!");
      elevatorMotor.set(-1);
      elevatorBreak.set(Value.kForward);
    }
  
    public void downElevatorMotor() {
      System.out.println("going down...");
      elevatorMotor.set(0.1);
      elevatorBreak.set(Value.kForward);
    }
  
    public void stopElevatorMotor() {
      elevatorMotor.set(0);
      elevatorBreak.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
