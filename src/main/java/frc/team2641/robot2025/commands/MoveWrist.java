package frc.team2641.robot2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.subsystems.superstructure.wrist.WristReal;

public class MoveWrist extends Command {
  private WristReal supS;

  public MoveWrist() {
    supS = WristReal.getInstance();
    addRequirements(supS);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double leftY = MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpLeftStickY(),0.05);
    supS.set(leftY*Constants.MotorSpeeds.wristSpeed);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
