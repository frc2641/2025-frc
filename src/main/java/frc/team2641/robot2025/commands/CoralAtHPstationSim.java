// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralAtHPstationSim extends InstantCommand {

  private double station;
  private int leftDeg;
  public CoralAtHPstationSim(boolean isLeftStation) {
    // Use addRequirements() here to declare subsystem dependencies.
    if(isLeftStation){
      station = 7.38;
      leftDeg = -40;
    }
      else
      {
      station = 0.62;
      leftDeg = 40;
      }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
SimulatedArena.getInstance()
  .addGamePieceProjectile(new ReefscapeCoralOnFly(
      // Obtain robot position from drive simulation
      new Translation2d(0.739,station),
      // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
      new Translation2d(0, 0),
      // Obtain robot speed from drive simulation
      new ChassisSpeeds(),
      // Obtain robot facing from drive simulation
      new Rotation2d(Degrees.of(leftDeg)),
      // The height at which the coral is ejected
      Meters.of(1.28),
      // The initial speed of the coral
      MetersPerSecond.of(2),
      // The coral is ejected at a 35-degree slope
      Degrees.of(-35))); }
  }

