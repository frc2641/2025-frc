package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Constants.ClimberConstants;

public class Winch extends SubsystemBase {
  private TalonFX motor;

  private static Winch instance;
  public static Winch getInstance() {
    if (instance == null) instance = new Winch();
    return instance;
  }

  private Winch() {
    configMotor();
  }

  public void stop() {
    motor.stopMotor();
  }

  public void extend() {
    // motor.getClosedLoopController().setReference(0, ControlType.kPosition);
    motor.set(-ClimberConstants.winchSpeed);

  }

  public void retract() {
    motor.set(ClimberConstants.winchSpeed);
  }

  private void configMotor() {
    motor = new TalonFX(Constants.CANConstants.winch);
    motor.setNeutralMode(NeutralModeValue.Brake);
  }
  
  @Override
  public void periodic() {
    if ((Math.abs(motor.getVelocity().getValueAsDouble()) < ClimberConstants.stallV) && (motor.getTorqueCurrent().getValueAsDouble() > ClimberConstants.stallI)){
      System.out.println("\n\n *** STALL DETECTED - WINCH *** \n\n");
    }
  }

  public TalonFX getMotor() {
    return motor;
  }
}