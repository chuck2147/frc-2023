package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class PIDNTValue {
  public PIDNTValue(double kP, double kI, double kD, double kF, TalonFX elevatorMotor, String name) {
    
    NTValue kPNTValue = new NTValue(kP, "kP " + name, "PID " + name);
    kPNTValue.addListener((val) -> {
      elevatorMotor.config_kP(0, val, 30);
    });
    elevatorMotor.config_kP(0, kPNTValue.value, 30);
   
    NTValue kINTValue = new NTValue(kI, "kI " + name, "PID " + name); 
    kINTValue.addListener((val) -> {
      elevatorMotor.config_kI(0, val, 30);
    });
    elevatorMotor.config_kI(0, kINTValue.value, 30);
  
    NTValue kDNTValue = new NTValue(kD, "kD " + name, "PID " + name);  
    kDNTValue.addListener((val) -> {
      elevatorMotor.config_kD(0, val, 30);
    });
    elevatorMotor.config_kD(0, kDNTValue.value, 30);
    
    NTValue kFNTValue = new NTValue(kF, "kF " + name, "PID " + name);
    kFNTValue.addListener((val) -> {
      elevatorMotor.config_kF(0, val, 30);
    });
    
    elevatorMotor.config_kF(0, kFNTValue.value, 30);
  }
}

  /*
  public PIDNTValue(double kP, double kI, double kD, PIDController pid, String name) {
    NTValue kPNTValue = new NTValue(kP, "kP " + name, "PID " + name);
    kPNTValue.entry.addListener((event) -> {
      pid.setP(val);
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    NTValue kINTValue = new NTValue(kI, "kI " + name, "PID " + name);
    kINTValue.entry.addListener((event) -> {
      pid.setI(val);
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    NTValue kDNTValue = new NTValue(kD, "kD " + name, "PID " + name);
    kDNTValue.entry.addListener((event) -> {
      pid.setD(val);
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }
}
*/