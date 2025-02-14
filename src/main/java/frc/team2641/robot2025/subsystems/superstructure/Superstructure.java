package frc.team2641.robot2025.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.helpers.ArmPosition;

public class Superstructure extends SubsystemBase {
  private Wrist wrist;
  private Elevator elevator;

  private ArmPosition pos;
  public boolean isAuto = false;

  public static enum ArmTargets {
    L1,
    L2,
    L3,
    L4,
    HUMAN_PLAYER,
    PROCESSOR,
    ALGAE_REMOVAL;
  }

  private static Superstructure instance;
  public static Superstructure getInstance() {
    if (instance == null)
      instance = new Superstructure();
    return instance;
  }

  private Superstructure() {
    wrist = Wrist.getInstance();
    elevator = Elevator.getInstance();
    pos = new ArmPosition(0, 0);
  }

  public void setPosition(ArmPosition pos) {
    this.pos = pos;
  }

  public void move() {
    wrist.setPosition(pos.getWrist());
    elevator.setPosition(pos.getElev());
  }

  public ArmPosition getPosition() {
    return pos;
  }

  public boolean atPos() {
    return (Math.abs(pos.getWrist() - wrist.getPosition()) < 0.5) && (Math.abs(pos.getElev() - elevator.getPosition()) < 0.5);
  }

  @Override
  public void periodic() {
  }
}