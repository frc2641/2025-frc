package frc.team2641.robot2025.commands.elevatorMoves;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Elevator;

public class Down extends Command {
  private Elevator elev;

  public Down() {
    this.elev = Elevator.getInstance();
    addRequirements(elev);
  }
  

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
   elev.down();
   }

  @Override
  public void end(boolean interrupted) {
    elev.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}