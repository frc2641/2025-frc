package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    wristMotor.configAllowableClosedloopError(0, Constants.IntakeGains.wristGains.kAllowableError, 30);

    wristMotor.config_kP(0, Constants.IntakeGains.wristGains.kP, 30);
    wristMotor.config_kI(0, Constants.IntakeGains.wristGains.kI, 30);
    wristMotor.config_kD(0, Constants.IntakeGains.wristGains.kD, 30);
    wristMotor.config_kF(0, Constants.IntakeGains.wristGains.kF, 30);
  }
}
