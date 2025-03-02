package frc.team2641.robot2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2641.robot2025.subsystems.superstructure.elevator.ElevatorSimulation;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import swervelib.parser.SwerveParser;
import org.ironmaple.simulation.*;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class Robot extends TimedRobot {
  private static Robot instance;
  private Command autoCommand;

  private static PowerDistribution pdh;
  private static PneumaticHub ph;
  public RobotContainer robotContainer;

  private Optional<SwerveDriveSimulation> driveSim;
  private static SimulatedArena arena;

  private Timer disabledTimer;

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  @Override
  public void robotInit() {
    // CameraServer.startAutomaticCapture(0);
    // CameraServer.startAutomaticCapture(1);
    robotContainer = new RobotContainer();
    disabledTimer = new Timer();

    for (int port = 5800; port <= 5807; port++) 
      PortForwarder.add(port, "10.26.41.25", port);

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
      arena = SimulatedArena.getInstance();
      driveSim = Drivetrain.getInstance().getSwerveDrive().getMapleSimDrive();
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DriveConstants.WHEEL_LOCK_TIME)) {
      robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  @Override
  public void autonomousInit() {
    robotContainer.setMotorBrake(true);

    autoCommand = robotContainer.getAutonomousCommand();
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    robotContainer.setMotorBrake(true);

    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    try {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    driveSim.get().setSimulationWorldPose(new Pose2d(new Translation2d(3,3), new Rotation2d(0)));

    robotContainer.elevSim = ElevatorSimulation.getInstance();
    // driveSim.get().setSimulationWorldPose(new Pose2d(new Translation2d(0,0), new Rotation2d(0)));

    arena.resetFieldForAuto();
    // arena.addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(4,2)));  

    // arena.addGamePiece(new ReefscapeCoralOnField(
    //   new Pose2d(2, 2, Rotation2d.fromDegrees(90))
    // ));
  
    Pose3d[] coral = arena.getGamePiecesArrayByType("Coral");
    Pose3d[] algae = arena.getGamePiecesArrayByType("Algae");

    robotContainer.coralPoses.accept(coral);
    robotContainer.algaePoses.accept(algae);
  }

  @Override
  public void simulationPeriodic() {
    robotContainer.updateSimulation();
  }

  public static PneumaticHub getPH() {
    return ph;
  }

  public static PowerDistribution getPDH() {
    return pdh;
  }

  public static SimulatedArena getArena() {
    return arena;
  }
}
