package frc.team2641.robot2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.helpers.ArmPosition;
import frc.team2641.robot2025.subsystems.Arm;

public class MoveArm extends Command {
  private Arm arm;

  public MoveArm() {
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (!arm.isAuto) {
      double ly = MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpLeftStickY(),0.5);
      double ry = MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpRightStickY(),0.5);
      arm.setPosition(new ArmPosition(arm.getPosition().getWrist() + 30 * ly, arm.getPosition().getElev() + 30 * ry));
    }

    arm.move();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
