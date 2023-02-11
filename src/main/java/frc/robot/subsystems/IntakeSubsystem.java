package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);     
    private CANSparkMax wristMotor = new CANSparkMax(Constants.WRIST_MOTOR_ID, MotorType.kBrushless);     

    public IntakeSubsystem() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        intakeMotor.setInverted(false); 

    }
    
    public void forwardIntakeMotor() {
        intakeMotor.set(1);
      }
    
      public void reverseIntakeMotor() {
        intakeMotor.set(-1);
      }

      public void stopIntakeMotor() {
        intakeMotor.set(0);
      }
    
      public void forwardWristMotor() {
        wristMotor.set(.5);
      }
    
      public void reverseWristMotor() {
        wristMotor.set(-.5);
      }
      
      public void stopWristMotor() {
        wristMotor.set(0);
      }


}
