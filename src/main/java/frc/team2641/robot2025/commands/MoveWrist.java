// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.subsystems.superstructure.wrist.WristReal;
import frc.team2641.robot2025.subsystems.superstructure.wrist.WristIO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveWrist extends Command {
  private WristReal supS;

  /** Creates a new MoveWrist. */
  public MoveWrist() {
    // Use addRequirements() here to declare subsystem dependencies.
    

    supS = WristReal.getInstance();
    addRequirements(supS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftY = MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpLeftStickY(),0.05);

    supS.set(leftY*Constants.MotorSpeeds.wristSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
