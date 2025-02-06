package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
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
    climber = new TalonFX(Constants.CAN.climber);
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

  @Override
  public void periodic() {
  }
}