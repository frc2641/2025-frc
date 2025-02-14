package frc.team2641.robot2025.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class Creep extends Command {
  private Drivetrain drivetrain;
  private int position;

  public Creep(int position) {
    drivetrain = Drivetrain.getInstance();
    this.position = position;
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
      if (position == 1)
        drivetrain.drive(new Translation2d(1.1, 0), -0.4, false);
      else
        drivetrain.drive(new Translation2d(1.1, 0), 0, false);
    } else {
      if (position == 1)
        drivetrain.drive(new Translation2d(1.1, 0), 0.4, false);
      else
        drivetrain.drive(new Translation2d(1.1, 0), 0, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new Translation2d(0, 0), 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
