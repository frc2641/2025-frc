package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;

public class Elevator extends SubsystemBase {
    private static Elevator instance;
    private TalonFX elevMotor;
    public static Elevator getInstance() {
      if (instance == null)
        instance = new Elevator();
      return instance;
    }   

    private Elevator()
    {
      // Needs ID set
      elevMotor = new TalonFX(Constants.CAN.elevMotor);
      configMotors();
      elevMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  
    public void stop() {
      elevMotor.stopMotor();
    }
    
    public void setPos(double pos){
      elevMotor.setPosition(pos);
    }

    public double getPos(){
      return elevMotor.getPosition().getValueAsDouble();
    }
  
  @Override
  public void periodic() {
  }

  private void configMotors() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.IntakeGains.elevatorGains.kP;
    slot0Configs.kI = Constants.IntakeGains.elevatorGains.kI;
    slot0Configs.kD = Constants.IntakeGains.elevatorGains.kD;

    elevMotor.getConfigurator().apply(slot0Configs);
  }
}
