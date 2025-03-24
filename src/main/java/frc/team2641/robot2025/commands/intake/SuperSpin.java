// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.intake.Intake;
import frc.team2641.robot2025.subsystems.intake.IntakeIO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SuperSpin extends Command {
  /** Creates a new SuperSpin. */
  private IntakeIO intake = Intake.getInstance();


  public SuperSpin(){

    addRequirements(intake);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intake.superSpin();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
