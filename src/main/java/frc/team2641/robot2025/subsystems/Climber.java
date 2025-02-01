package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.Constants;

public class Climber extends SubsystemBase {
  private static Climber instance;
  private boolean extended = false;

    private TalonFX climber;

  public static Climber getInstance() {
    if (instance == null)
      instance = new Climber();
    return instance;
  }


  private Climber() {
    // Needs ID set
    climber = new TalonFX(-1);
  }

  public void up(){
    climber.set(Constants.MotorSpeeds.climbSpeed);
  }

  public void down()
  {
    climber.set(-Constants.MotorSpeeds.climbSpeed);
  }

  public void stop() {
    climber.stopMotor();
  }

  public boolean isAt(double pos){
    return Math.abs(pos-climber.getPosition().getValueAsDouble())<0.5;
  }


  @Override
  public void periodic() {
  }
}