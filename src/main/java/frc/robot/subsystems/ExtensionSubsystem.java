// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtensionSubsystem extends SubsystemBase { 
  private CANSparkMax extensionMotor = new CANSparkMax(Constants.EXTENSION_MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder extension_encoder;
  private SparkMaxPIDController extension_pidController;

// PID coefficients............................................
  double kP = 0.5; 
  double kI = 0;
  double kD = 0; 
  double kIz = 0; 
  double kFF = 0; 
  double kMaxOutput = 1; 
  double kMinOutput = -1;

//PID Setpoint....................................................
  double l3Extension = 80;
  double l2Extension = 50;
  double humanExtension = 40;
  double intakeExtension = 30;
  double stowedExtension = 0;


  public ExtensionSubsystem() {

    extensionMotor.restoreFactoryDefaults();
    extensionMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    extensionMotor.setInverted(false); 
  
    extensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    extensionMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
  
    extensionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15); //what is 15?
    extensionMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
  
    extension_encoder = extensionMotor.getEncoder();
         
    extension_pidController = extensionMotor.getPIDController();
    
    // set PID coefficients
    extension_pidController.setP(kP); 
    extension_pidController.setI(kI);
    extension_pidController.setD(kD);
    extension_pidController.setIZone(kIz);
    extension_pidController.setFF(kFF);
    extension_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

public void l3Extension () {
  extension_pidController.setReference(l3Extension, CANSparkMax.ControlType.kPosition);
  }

public void l2Extension () {
  extension_pidController.setReference(l2Extension, CANSparkMax.ControlType.kPosition);
  }

public void intakeExtension () {
   extension_pidController.setReference(l2Extension, CANSparkMax.ControlType.kPosition);
   }

public void humanExtension () {
   extension_pidController.setReference(l2Extension, CANSparkMax.ControlType.kPosition);
   }

public void stowedExtension () {
  extension_pidController.setReference(stowedExtension, CANSparkMax.ControlType.kPosition);
  }

public void resetEncoder() {
  extension_encoder.setPosition(0);
  }

  public void forwardExtensionMotor() {
    extensionMotor.set(0.2);
  }

  public void reverseExtensionMotor() {
    extensionMotor.set(-.2);
  }

  public void stopExtensionMotor() {
    extensionMotor.set(0);
}

public double getExtensionPosition() {
  return extension_encoder.getPosition();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Extension Position", getExtensionPosition());

  }
}
