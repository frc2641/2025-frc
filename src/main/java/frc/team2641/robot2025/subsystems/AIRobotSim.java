// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.subsystems;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.RobotContainer;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;
public class AIRobotSim extends SubsystemBase {
  /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
  public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
      new Pose2d(-6, 0, new Rotation2d()),
      new Pose2d(-5, 0, new Rotation2d()),
      new Pose2d(-4, 0, new Rotation2d()),
      new Pose2d(-3, 0, new Rotation2d()),
      new Pose2d(-2, 0, new Rotation2d())
  };

  private final Pose2d queeningPose;
  private final int id;
    private final SimplifiedSwerveDriveSimulation driveSim;

  public AIRobotInSimulation(int id) {
      this.id = id;
      this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
      this.driveSim = Drivetrain.getInstance().getSwerveDrive().getMapleSimDrive().get();

      SimulatedArena.getInstance().addDriveTrainSimulation(
          driveSim.getDriveTrainSimulation()
      );
  }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
