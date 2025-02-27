package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;

public class Climber extends SubsystemBase {
  private TalonFX climber;
  
  private static Climber instance;
  public static Climber getInstance() {
    if (instance == null)
      instance = new Climber();
    return instance;
  }

  private Climber() {
    // TODO: Set ID
    // climber = new TalonFX(Constants.CAN.climber);
    configMotors();    
  }

  public void extend() {
    climber.set(Constants.MotorSpeeds.climbSpeed);
  }

  public void retract() {
    climber.set(-Constants.MotorSpeeds.climbSpeed);
  }

  public void stop() {
    climber.stopMotor();
  }

  private void configMotors() {
    // TODO: Set ID
    climber = new TalonFX(Constants.CAN.climber);

    // Slot0Configs slot0Configs = new Slot0Configs();
    // slot0Configs.kP = Constants.IntakeGains.climbGains.kP;
    // slot0Configs.kD = Constants.IntakeGains.climbGains.kD;
    // slot0Configs.kI = Constants.IntakeGains.climbGains.kI;

    // climber.getConfigurator().apply(slot0Configs);
    
    climber.setNeutralMode(NeutralModeValue.Brake);
  }
  

  @Override
  public void periodic() {
  }
}