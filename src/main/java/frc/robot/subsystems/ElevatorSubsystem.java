// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
  double kP = 1;
  double kI = 0;
  double kD = 0.001;
  double kF = 0;

  // PID Setpoint....................................................
  double l3ElevatorPosition = 135000; // 153000
  double l2ElevatorPosition = 87000; // 87000
  double humanElevatorPosition = 110000; // 170518
  double stowedElevatorPosition = 0; // starting configuration set when robot turned on
  double intakeElevatorPosition = -27700; // negative because will be lower than starting configuration
  boolean hasResetEncoder = false; 
  Timer resetEncoderTimer = null;

  // Shuffleboard entries...........................................
  // ShuffleboardTab tab = Shuffleboard.getTab("NTValues");
  // Topic l2ElevatorPositionEntry = tab.add("L2 Elevator Position",
  // 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry().getTopic();
  // Topic l3ElevatorPositionEntry = tab.add("L3 Elevator Position",
  // 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry().getTopic();
  // Topic humanElevatorPositionEntry = tab.add("Human Elevator Position",
  // 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry().getTopic();
  // Topic intakeElevatorPositionEntry = tab.add("Intake Elevator Position",
  // 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry().getTopic();

  public ElevatorSubsystem() {

    elevatorMotor.configFactoryDefault();
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    elevatorMotor.setInverted(TalonFXInvertType.CounterClockwise);//Clockwise for Alpha / CounterClockwise for Comp // subject to change
    elevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    elevatorMotor.setSensorPhase(true);// check
    elevatorMotor.configPeakOutputForward(1);
    elevatorMotor.configPeakOutputReverse(-0.92);

    /* set up followers */
    elevatorMotorFollower.configFactoryDefault();
    elevatorMotorFollower.setNeutralMode(NeutralMode.Brake);
    elevatorMotorFollower.setInverted(TalonFXInvertType.CounterClockwise);// check
    elevatorMotorFollower.follow(elevatorMotor);

    // set PID coefficients
    elevatorMotor.config_kP(0, kP, 30);
    elevatorMotor.config_kI(0, kI, 30);
    elevatorMotor.config_kD(0, kD, 30);
    elevatorMotor.config_kF(0, kF, 30);

    new PIDNTValue(kP, kI, kD, kF, elevatorMotor, "Elevator Motor");
  }

  public void l3Elevator() {
    if (hasResetEncoder == true) {
      elevatorMotor.set(ControlMode.Position, l3ElevatorPosition);
      elevatorBreak.set(Value.kForward);
    }
  }

  public void l2Elevator() {
    if (hasResetEncoder == true) {
      elevatorMotor.set(ControlMode.Position, l2ElevatorPosition);
      elevatorBreak.set(Value.kForward);
    }
  }

  public void stowedElevator() {
    if (hasResetEncoder == true) {
      elevatorMotor.set(ControlMode.Position, stowedElevatorPosition);
      elevatorBreak.set(Value.kForward);
    }
  }

  public void intakeElevator() {
    if (hasResetEncoder == true) {
      elevatorMotor.set(ControlMode.Position, intakeElevatorPosition);
      elevatorBreak.set(Value.kForward);
    }
  }

  public void humanElevator() {
    if (hasResetEncoder == true) {
      elevatorMotor.set(ControlMode.Position, humanElevatorPosition);
      elevatorBreak.set(Value.kForward);
    }
  }

  public void upElevatorMotor() {
    if (hasResetEncoder == true) {
      elevatorMotor.set(ControlMode.PercentOutput, 1);
      elevatorBreak.set(Value.kForward);
    }
  }

  public void downElevatorMotor() {
    if (hasResetEncoder == true) {
      elevatorMotor.set(ControlMode.PercentOutput, -0.92);
      elevatorBreak.set(Value.kForward);
    }
  }

  public void stopElevatorMotor() {
    if (hasResetEncoder == true) {
      elevatorMotor.set(ControlMode.PercentOutput, 0);
      elevatorBreak.set(Value.kReverse);
    }
  }

  public double getElevatorEncoder() {
    return elevatorMotor.getSelectedSensorVelocity(); //getSelectedSensorPosition should be right...
  }

  public void resetElevatorEncoder() {
    elevatorMotor.set(ControlMode.Position, stowedElevatorPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!hasResetEncoder) {
      if (resetEncoderTimer == null) {
        resetEncoderTimer= new Timer();
        resetEncoderTimer.restart();
      }

      if (resetEncoderTimer.get() > 0.1) {
        hasResetEncoder = true;
        resetEncoderTimer = null;
        stopElevatorMotor();
        resetElevatorEncoder();

      } else {
        elevatorMotor.set(ControlMode.PercentOutput, 0.000);
        elevatorBreak.set(Value.kForward);

      }
    }

    SmartDashboard.putNumber("Elevator Encoder", getElevatorEncoder());

    // l2ElevatorPositionEntry.genericPublish("double").setDouble(l2ElevatorPosition);
    // l3ElevatorPositionEntry.genericPublish("double").setDouble(l3ElevatorPosition);
    // humanElevatorPositionEntry.genericPublish("double").setDouble(humanElevatorPosition);
    // intakeElevatorPositionEntry.genericPublish("double").setDouble(intakeElevatorPosition);

  }

  @Override
  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation
  }
}
