// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.RobotContainer;
import frc.team2641.robot2025.helpers.ArmPos;
import frc.team2641.robot2025.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveArm extends Command {

  private Arm arm;
  /** Creates a new MoveArm. */
  public MoveArm() {
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(!arm.isAuto){
      double ly = MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpLeftJoy(),0.5);
      double ry = MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpRightJoy(),0.5);
      arm.changeTarget(new ArmPos(arm.getPosition().getWrist()+30*ly,arm.getPosition().getElev()+30*ry));
      }
      arm.move();
  }


  // Called once the command ends or is interrupd.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
