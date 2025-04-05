package frc.team2641.robot2025.commands.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Winch;

public class Wrap extends Command {
  private Winch winch = Winch.getInstance();
  private boolean forwards;

  public Wrap(boolean forwards) {
    this.forwards = forwards;
    addRequirements(winch);
  }

  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    if (forwards)
      winch.extend();
    else
      winch.retract();
  }

  @Override
  public void end(boolean interrupted) {
    winch.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
