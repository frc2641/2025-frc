package frc.team2641.robot2025.commands.spinIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Intake;

public class Out extends Command {
  private Intake intake;

  public Out() {
    this.intake = Intake.getInstance();
    addRequirements(intake);
  }
  

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    
    intake.out();
    
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