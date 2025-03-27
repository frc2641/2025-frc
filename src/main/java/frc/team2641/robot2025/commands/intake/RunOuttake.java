package frc.team2641.robot2025.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Intake;

public class RunOuttake extends Command {
  private Intake intake = Intake.getInstance();

  public RunOuttake() {
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.secondSpin();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
