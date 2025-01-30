package frc.team2641.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Intake;

public class SpinIntake extends Command {
  private Intake intake;
  private boolean takingIn;

  public SpinIntake(boolean takingIn) {
    this.takingIn = takingIn;
    this.intake = Intake.getInstance();
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    if(takingIn)
    intake.in();
    else
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