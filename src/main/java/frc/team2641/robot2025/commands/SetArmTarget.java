package frc.team2641.robot2025.commands;

import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.helpers.ArmPosition;
import frc.team2641.robot2025.subsystems.superstructure.Superstructure;
import edu.wpi.first.wpilibj2.command.Command;
  
public class SetArmTarget extends Command {
  private ArmPosition pos;
  private Superstructure arm;
  
  public SetArmTarget(ArmPosition pos) {
    this.pos = pos;
    arm = Superstructure.getInstance();
    addRequirements(arm);
  }

  public SetArmTarget(Superstructure.ArmTargets target) {
    switch (target) {
      case L1:
        this.pos = Constants.ArmPositions.L1;
        break;
      case L2:
        this.pos = Constants.ArmPositions.L2;
        break;
      case L3:
        this.pos = Constants.ArmPositions.L3;
        break;
      case L4:
        this.pos = Constants.ArmPositions.L4;
        break;
      case HUMAN_PLAYER:
        this.pos = Constants.ArmPositions.humanPlayer;
        break;
      case PROCESSOR:
        this.pos = Constants.ArmPositions.processor;
        break;
      case ALGAE_REMOVAL:
        this.pos = Constants.ArmPositions.algaeRemoval;
        break;
      default:
        this.pos = new ArmPosition(0, 0);
        break;
    }
  }

  @Override
  public void initialize() {
    arm.isAuto = true;
    arm.setPosition(pos);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    arm.isAuto = false;
  }

  @Override
  public boolean isFinished() {
    return arm.atPos();
  }
}
