package frc.team2641.robot2025.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class Creep extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Timer timer = new Timer();

  public Creep() {
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    drivetrain.drive(new Translation2d(-1.1, 0), 0, false);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new Translation2d(0, 0), 0, false);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2);
  }
}
