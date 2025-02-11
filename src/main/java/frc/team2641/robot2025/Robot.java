package frc.team2641.robot2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2641.robot2025.subsystems.Swerve;

// import frc.team2641.robot2025.subsystems.Pneumatics;
import java.io.File;
import java.io.IOException;
import swervelib.parser.SwerveParser;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.ironmaple.simulation.*;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation.SelfControlledModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;

public class Robot extends TimedRobot {
  private static Robot instance;
  private Command autoCommand;

  // private static Pneumatics pneumatics;

  private static PowerDistribution pdh;
  private static PneumaticHub ph;
  public RobotContainer robotContainer;

  private Timer disabledTimer;

  StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
    .getStructArrayTopic("FieldElements/Alage", Pose3d.struct)
    .publish();

  StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
    .getStructArrayTopic("FieldElements/Coral", Pose3d.struct)
    .publish();

  public Robot() {
    SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(4,2)));
    
    Pose3d[] algae = SimulatedArena.getInstance().getGamePiecesArrayByType("Algae");
    algaePoses.accept(algae);

SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
    // We must specify a heading since the coral is a tube
    new Pose2d(2, 2, Rotation2d.fromDegrees(90))));


    Pose3d[] coral = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
    coralPoses.accept(coral);

    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  @Override
  public void robotInit() {
    // CameraServer.startAutomaticCapture(0);
    // CameraServer.startAutomaticCapture(1);
    // pneumatics = Pneumatics.getInstance();
    robotContainer = new RobotContainer();
    disabledTimer = new Timer();

    for (int port = 5800; port <= 5807; port++) 
      PortForwarder.add(port, "10.26.41.25", port);

    if (isSimulation())
      DriverStation.silenceJoystickConnectionWarning(true);
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
    // pneumatics.disable();
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

    // pneumatics.enable();
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

    // pneumatics.enable();
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
}
