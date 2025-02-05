package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;

public class Wrist extends SubsystemBase {
    private static Wrist instance;
    private TalonFX wristMotor;
    public static Wrist getInstance() {
      if (instance == null)
        instance = new Wrist();
      return instance;
    }   

    private Wrist()
    {
      // Needs ID set
      wristMotor = new TalonFX(Constants.CAN.elevMotor);
      configMotors();
      wristMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  
    public void stop() {
      wristMotor.stopMotor();
    }
    
    public void setPos(double pos){
      wristMotor.setPosition(pos);
    }

    public double getPos(){
      return wristMotor.getPosition().getValueAsDouble();
    }
  
  @Override
  public void periodic() {
  }

  private void configMotors() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.IntakeGains.wristGains.kP;
    slot0Configs.kI = Constants.IntakeGains.wristGains.kI;
    slot0Configs.kD = Constants.IntakeGains.wristGains.kD;

    wristMotor.getConfigurator().apply(slot0Configs);
  }
}
