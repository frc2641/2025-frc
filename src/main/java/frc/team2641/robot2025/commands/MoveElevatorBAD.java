package frc.team2641.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Elevator;

public class MoveElevatorBAD extends Command {
  private Elevator elev;
  private boolean up;

  public MoveElevatorBAD(boolean up) {
    this.up = up;
    this.elev = Elevator.getInstance();
    addRequirements(elev);
  }
  

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
   if(up)
   elev.up();
   else
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