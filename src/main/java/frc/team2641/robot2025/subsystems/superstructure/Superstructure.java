package frc.team2641.robot2025.subsystems.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.helpers.ArmPosition;
import frc.team2641.robot2025.subsystems.superstructure.wrist.WristReal;
import frc.team2641.robot2025.subsystems.superstructure.elevator.Elevator;
import frc.team2641.robot2025.subsystems.superstructure.elevator.ElevatorIO;
import frc.team2641.robot2025.subsystems.superstructure.wrist.WristIO;

public class Superstructure extends SubsystemBase {
  private WristIO wrist;
  private ElevatorIO elevator;

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
    if (instance == null) instance = new Superstructure();
    return instance;
  }

  private Superstructure() {
    wrist = WristReal.getInstance();
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
    SmartDashboard.putNumber("Arm Elev Pos ", pos.getElev());
    SmartDashboard.putNumber("Arm Wrist Pos ", pos.getWrist());
  }
}