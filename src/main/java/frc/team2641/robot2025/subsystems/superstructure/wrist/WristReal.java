package frc.team2641.robot2025.subsystems.superstructure.wrist;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Robot;

public class WristReal extends SubsystemBase implements WristIO {
  private TalonFX motor;

  private static WristReal instance;
  public static WristReal getInstance() {
      if (instance == null) {
        instance = new WristReal();
      }
      return instance;
    }
  

  private WristReal() {
    configMotors();
  }

  public void stop() {
    motor.stopMotor();
  }

  public void set(double speed) {
    motor.set(speed);
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
    if((Math.abs(motor.getVelocity().getValue().baseUnitMagnitude())<0.1)&&(motor.getTorqueCurrent().getValue().baseUnitMagnitude()>30)){
      // stop();
      System.out.println("\n Stall detected - Wrist Motor Stopped \n");
    }
  }
}