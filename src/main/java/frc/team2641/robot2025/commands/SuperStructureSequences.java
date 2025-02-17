package frc.team2641.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.helpers.ArmPosition;
import frc.team2641.robot2025.subsystems.superstructure.Superstructure;;

public class SuperStructureSequences {
    public static Command processor(){
        return Commands.sequence(
            new SetArmTarget(new ArmPosition(Constants.ArmPositions.processor.getWrist(), Superstructure.getInstance().getPosition().getElev())),
            new MoveArm(),
            new SetArmTarget(Superstructure.ArmTargets.PROCESSOR)
        );
    }
    public static Command l1(){
        return Commands.sequence(
            new SetArmTarget(new ArmPosition(Constants.ArmPositions.L1.getWrist(), Superstructure.getInstance().getPosition().getElev())),
            new MoveArm(),
            new SetArmTarget(Superstructure.ArmTargets.L1)
        );
    }
}
