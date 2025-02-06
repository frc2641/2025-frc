package frc.team2641.robot2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.helpers.ArmPosition;

public class Arm extends SubsystemBase {
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
    ALGAE_REMOVAL
  };

  private static Arm instance;
  public static Arm getInstance() {
    if (instance == null)
      instance = new Arm();
    return instance;
  }

  private Arm() {
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