package frc.team2641.robot2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Climber extends SubsystemBase {
  private static Climber instance;

  public static Climber getInstance() {
    if (instance == null)
      instance = new Climber();
    return instance;
  }

  private DoubleSolenoid climber = Robot.getPH().makeDoubleSolenoid(10, 9);

  private Climber() {
    lower();
  }

  public void depressurize() {
    climber.set(DoubleSolenoid.Value.kOff);
  }

  public void lower() {
    climber.set(DoubleSolenoid.Value.kReverse);
  }

  public void raise() {
    climber.set(DoubleSolenoid.Value.kForward);
  }

  public void toggle() {
    if (get().equals(DoubleSolenoid.Value.kReverse))
      raise();
    else
      lower();
  }

  public DoubleSolenoid.Value get() {
    return climber.get();
  }

  @Override
  public void periodic() {
  }
}