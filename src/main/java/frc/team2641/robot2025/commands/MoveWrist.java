package frc.team2641.robot2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.subsystems.superstructure.wrist.WristReal;

public class MoveWrist extends Command {
  private WristReal wrist;

  public MoveWrist() {
    wrist = WristReal.getInstance();
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double leftY = MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpLeftStickY(), 0.05);
    double output = wrist.getSetpoint() + leftY * Constants.IntakeConstants.wristSpeed;
    wrist.goTo(output);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
