package frc.team2641.robot2025.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.subsystems.intake.Intake;
import frc.team2641.robot2025.subsystems.intake.IntakeIO;
import frc.team2641.robot2025.subsystems.intake.IntakeIOSim;

public class RunOuttake extends Command {
  private IntakeIO intake;

  public RunOuttake() {
    this.intake = Intake.getInstance();
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.secondSpin();
  }

  @Override
  public void execute() {
    
    if (Robot.isSimulation()) {
      if (IntakeIOSim.getInstance().preventDoubleGamePiece()) end(false);
    }
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
