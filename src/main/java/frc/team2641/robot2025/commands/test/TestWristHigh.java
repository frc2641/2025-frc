package frc.team2641.robot2025.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.superstructure.wrist.WristReal;
import frc.team2641.robot2025.Constants.IntakeConstants;

public class TestWristHigh extends Command {
  WristReal wrist = WristReal.getInstance();

  public TestWristHigh() {
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    wrist.goTo(IntakeConstants.wristMaxPos);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return wrist.getPosition() >= IntakeConstants.wristMaxPos;
  }
}
