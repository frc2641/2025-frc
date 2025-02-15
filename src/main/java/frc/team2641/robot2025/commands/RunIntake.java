package frc.team2641.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.RobotContainer;
import frc.team2641.robot2025.subsystems.intake.Intake;
import frc.team2641.robot2025.subsystems.intake.IntakeIO;
import frc.team2641.robot2025.subsystems.intake.IntakeIOSim;

public class RunIntake extends Command {
  private IntakeIO intake;

  public RunIntake() {
    this.intake = Intake.getInstance();
    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.spin();
    if(Robot.isSimulation())
      if(IntakeIOSim.getInstance().preventDoubleGamePiece())
        end(false);
      
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
