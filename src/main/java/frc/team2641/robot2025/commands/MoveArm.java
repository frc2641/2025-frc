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
    System.out.print("Arm Position: ");
    System.out.println(arm.getPosition());
    if (!arm.isAuto) {
      double leftY = MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpLeftStickY(),0.05);
      double rightY= MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpRightStickY(),0.05);
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
