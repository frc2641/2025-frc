package frc.team2641.robot2025;

import org.ironmaple.simulation.SimulatedArena;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team2641.robot2025.Constants.ELEVNUM;
import frc.team2641.robot2025.Constants.OperatorConstants;
import frc.team2641.robot2025.commands.auto.*;
import frc.team2641.robot2025.commands.climbing.Climb;
import frc.team2641.robot2025.commands.climbing.Wrap;
import frc.team2641.robot2025.commands.elevator.MoveElev;
import frc.team2641.robot2025.commands.elevator.SetElev;
import frc.team2641.robot2025.commands.intake.RunIntake;
import frc.team2641.robot2025.commands.intake.RunOuttake;
import frc.team2641.robot2025.commands.shifts.*;
import frc.team2641.robot2025.commands.sim.CoralAtHPstationSim;
import frc.team2641.robot2025.subsystems.elevator.Elevator;
import frc.team2641.robot2025.subsystems.elevator.ElevatorIO;
import frc.team2641.robot2025.subsystems.elevator.ElevatorSimulation;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class RobotContainer {
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final ElevatorIO elev = Elevator.getInstance();

  CommandXboxController driverGamepad = new CommandXboxController(0);
  CommandXboxController operatorGamepad = new CommandXboxController(1);

  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  Command driveCommand;
  Command driveSim;

  BooleanPublisher alignmentPub;
  BooleanSubscriber alignmentSub;

  BooleanPublisher sniperPub;
  BooleanSubscriber sniperSub;

  BooleanPublisher robotPub;
  BooleanSubscriber robotSub;
  
  DoublePublisher angularVelocityPub;
  DoubleSubscriber angularVelocitySub;

  private SimulatedArena arena;
  ElevatorSimulation elevSim;

  StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
    .getStructArrayTopic("FieldElements/Alage", Pose3d.struct)
    .publish();

  StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
    .getStructArrayTopic("FieldElements/Coral", Pose3d.struct)
    .publish();

  public RobotContainer() {
    driverGamepad.a().whileTrue(new AutoAngle(1, false));
    driverGamepad.b().whileTrue(new AutoAngle(2, false));
    driverGamepad.x().whileTrue(new AutoAngle(3, false));
    driverGamepad.y().whileTrue(new AutoAngle(4, false));
    driverGamepad.leftBumper().whileTrue(new LimelightTracking());
    driverGamepad.leftTrigger().whileTrue(new SniperMode());
    driverGamepad.rightTrigger().whileTrue(new RobotRelative());
    driverGamepad.start().onTrue(new InstantCommand(drivetrain::zeroGyro));

    driverGamepad.povUp().whileTrue(new Climb(true));
    driverGamepad.povDown().whileTrue(new Climb(false));
    driverGamepad.povLeft().whileTrue(new Wrap(true));
    driverGamepad.povRight().whileTrue(new Wrap(false));

    operatorGamepad.leftTrigger().whileTrue(new RunIntake());
    operatorGamepad.rightTrigger().whileTrue(new RunOuttake());
    operatorGamepad.a().onTrue(new SetElev(ELEVNUM.L1));
    operatorGamepad.b().onTrue(new SetElev(ELEVNUM.L2));
    operatorGamepad.x().onTrue(new SetElev(ELEVNUM.L3));
    operatorGamepad.y().onTrue(new SetElev(ELEVNUM.L4));
    operatorGamepad.start().onTrue(new SetElev(ELEVNUM.HP));
    operatorGamepad.back().onTrue(new SetElev(0));

    if (Robot.isSimulation()){
      operatorGamepad.start().onTrue(new CoralAtHPstationSim(false));
      operatorGamepad.back().onTrue(new CoralAtHPstationSim(true));
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    if (Robot.isSimulation()) arena = Robot.getArena();
    alignmentPub = table.getBooleanTopic("autoAlign").publish();
    alignmentPub.set(false);
    alignmentSub = table.getBooleanTopic("autoAlign").subscribe(false);

    angularVelocityPub = table.getDoubleTopic("angularVelocity").publish();
    angularVelocityPub.set(0);
    angularVelocitySub = table.getDoubleTopic("angularVelocity").subscribe(0);

    sniperPub = table.getBooleanTopic("sniperMode").publish();
    sniperPub.set(false);
    sniperSub = table.getBooleanTopic("sniperMode").subscribe(false);

    robotPub = table.getBooleanTopic("robotRelative").publish();
    robotPub.set(false);
    robotSub = table.getBooleanTopic("robotRelative").subscribe(false);

    driveCommand = drivetrain.driveCommand(
      () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepad.getLeftY() * Constants.SNIPER_MODE : -driverGamepad.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepad.getLeftX() * Constants.SNIPER_MODE : -driverGamepad.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> alignmentSub.get() ? angularVelocitySub.get() : sniperSub.get() ? -driverGamepad.getRightX() * 0.75 : -driverGamepad.getRightX(),
      () -> robotSub.get());

    drivetrain.setDefaultCommand(driveCommand);
    elev.setDefaultCommand(new MoveElev());

    NamedCommands.registerCommand("creep", new Creep(0));
    NamedCommands.registerCommand("creepAmp", new Creep(1));
    NamedCommands.registerCommand("angleSource", new AutoAngle(4, true));

    autoChooser.setDefaultOption("Shoot Creep", "Shoot Creep");
    // SmartDashboard.putData("Auto", autoChooser);
    Autos.publishAll();
  }

  public Command getAutonomousCommand() {
    // return drivetrain.getAutonomousCommand(autoChooser.getSelected());
    return Autos.getAutoCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivetrain.setMotorBrake(brake);
  }

  public double getOpLeftStickY() {
    SmartDashboard.putNumber("getOpLeftStickY", operatorGamepad.getLeftY());
    return operatorGamepad.getLeftY();
  }

  public double getOpRightStickY() {
    SmartDashboard.putNumber("getOpRightStickY", operatorGamepad.getRightY());
    return operatorGamepad.getRightY();
  }

  public void updateSimulation() {
    if (Robot.isReal()) return;
    if (arena == null) arena = Robot.getArena();
    
    Pose3d[] algae = arena.getGamePiecesArrayByType("Algae");
    algaePoses.accept(algae);

    Pose3d[] coral = arena.getGamePiecesArrayByType("Coral");
    coralPoses.accept(coral);

    arena.simulationPeriodic();
  }
}
