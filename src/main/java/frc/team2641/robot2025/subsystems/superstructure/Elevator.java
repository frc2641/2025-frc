package frc.team2641.robot2025.subsystems.superstructure;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;

public class Elevator extends SubsystemBase {
  private TalonFX motor;
  
  private static Elevator instance;
  public static Elevator getInstance() {
    if (instance == null)
      instance = new Elevator();
    return instance;
  }   

  private Elevator() {
    configMotors();
  }

  public void stop() {
    motor.stopMotor();
  }
  
  public void setPosition(double pos){
    motor.setPosition(pos);
  }

  public double getPosition(){
    return motor.getPosition().getValueAsDouble();
  }

  private void configMotors() {
    // TODO: Set ID
    motor = new TalonFX(Constants.CAN.elevator);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.IntakeGains.elevatorGains.kP;
    slot0Configs.kI = Constants.IntakeGains.elevatorGains.kI;
    slot0Configs.kD = Constants.IntakeGains.elevatorGains.kD;

    motor.getConfigurator().apply(slot0Configs);
    
    motor.setNeutralMode(NeutralModeValue.Brake);
  }
  
  @Override
  public void periodic() {
  }
}
