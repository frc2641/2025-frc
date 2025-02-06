package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;

public class Wrist extends SubsystemBase {
  private TalonFX motor;

  private static Wrist instance;
  public static Wrist getInstance() {
    if (instance == null)
      instance = new Wrist();
    return instance;
  }   

  private Wrist() {
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
    motor = new TalonFX(Constants.CAN.wrist);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.IntakeGains.wristGains.kP;
    slot0Configs.kI = Constants.IntakeGains.wristGains.kI;
    slot0Configs.kD = Constants.IntakeGains.wristGains.kD;

    motor.getConfigurator().apply(slot0Configs);

    motor.setNeutralMode(NeutralModeValue.Brake);
  }
  
  @Override
  public void periodic() {
  }
}
