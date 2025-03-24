package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants.CANConstants;
import frc.team2641.robot2025.Constants.ClimberConstants;

public class Climber extends SubsystemBase  {
  private TalonFX motor;

  private static Climber instance;
  public static Climber getInstance() {
    if (instance == null) instance = new Climber();
    return instance;
  }

  private Climber() {
    configMotor();
  }

  public void stop() {
    motor.stopMotor();
  }

  public void extend() {
    motor.set(ClimberConstants.climberSpeed);
  }

  public void retract() {
    motor.set(-ClimberConstants.climberSpeed);
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  private void configMotor() {
motor = new TalonFX(CANConstants.climber);

motor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    if ((Math.abs(motor.getVelocity().getValueAsDouble()) < ClimberConstants.stallV) && (motor.getStatorCurrent().getValueAsDouble() > ClimberConstants.stallI)){
      System.out.println("\n\n *** STALL DETECTED - CLIMBER *** \n\n");
      // stop();
    }
  }

  public TalonFX getMotor() {
    return motor;
  }
}