// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.commands;

import frc.team2641.robot2025.subsystems.Arm;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.helpers.ArmPos;

import edu.wpi.first.wpilibj2.command.Command;
  
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmPosition extends Command {

  private ArmPos pos;
  private Arm arm;
  
  public SetArmPosition(ArmPos pos) {
    this.pos = pos;
    arm = Arm.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SetArmPosition(Arm.switcher pos){
    switch (pos) {
      case L1:
        this.pos = Constants.ArmPositions.L1;
        break;
        case L2:
        this.pos = Constants.ArmPositions.L2;
        
        break;
        case L3:
        this.pos = Constants.ArmPositions.L3;

        break;
        case L4:
        this.pos = Constants.ArmPositions.L4;

        break;
        case HUMAN_PLAYER:
        this.pos = Constants.ArmPositions.humanPlayer;

        break;
        case PROCESSOR:
        this.pos = Constants.ArmPositions.processor;

        break;

        case ALGAE_REMOVAL:
        this.pos = Constants.ArmPositions.algaeRemoval;
        break;
      default:
        this.pos = new ArmPos(Integer.MIN_VALUE, Integer.MIN_VALUE);
        break;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.isAuto = true;
    arm.setTargetPosition(pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.isAuto = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atPos();
  }
}
