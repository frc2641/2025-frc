package frc.team2641.robot2025.commands.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Climber;

public class Climb extends Command {
  private Climber climber = Climber.getInstance();
  private boolean forwards;

  public Climb(boolean forwards) {
    this.forwards = forwards;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    if(forwards)
      climber.extend();
    else
    climber.retract();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
