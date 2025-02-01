package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  
  @Override
  public void periodic() {
  }

  private void configMotors() {
    elevMotor.configAllowableClosedloopError(0, Constants.IntakeGains.elevatorGains.kAllowableError, 30);

    elevMotor.config_kP(0, Constants.IntakeGains.elevatorGains.kP, 30);
    elevMotor.config_kI(0, Constants.IntakeGains.elevatorGains.kI, 30);
    elevMotor.config_kD(0, Constants.IntakeGains.elevatorGains.kD, 30);
    elevMotor.config_kF(0, Constants.IntakeGains.elevatorGains.kF, 30);
  }
}
