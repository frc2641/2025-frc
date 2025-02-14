package frc.team2641.robot2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.helpers.ArmPosition;
import frc.team2641.robot2025.subsystems.superstructure.Superstructure;

public class MoveArm extends Command {
  private Superstructure arm;

  public MoveArm() {
    arm = Superstructure.getInstance();
    addRequirements(arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (!arm.isAuto) {
      double leftY = MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpLeftStickY(),0.5);
      double rightY= MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpRightStickY(),0.5);
      arm.setPosition(new ArmPosition(arm.getPosition().getWrist() + 30 * leftY, arm.getPosition().getElev() + 30 * rightY));
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
