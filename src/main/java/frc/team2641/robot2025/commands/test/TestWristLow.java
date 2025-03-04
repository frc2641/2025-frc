package frc.team2641.robot2025.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.superstructure.wrist.WristReal;
import frc.team2641.robot2025.Constants.WristConstants;

public class TestWristLow extends Command {
  WristReal wrist = WristReal.getInstance();

  public TestWristLow() {
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    wrist.goTo(WristConstants.minPos);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return wrist.getPosition() <= WristConstants.minPos;
  }
}
