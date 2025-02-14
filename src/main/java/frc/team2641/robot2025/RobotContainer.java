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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team2641.robot2025.Constants.OperatorConstants;
import frc.team2641.robot2025.commands.*;
import frc.team2641.robot2025.commands.auto.*;
import frc.team2641.robot2025.commands.shifts.*;
// import frc.team2641.robot2025.subsystems.superstructure.Superstructure;
import frc.team2641.robot2025.subsystems.superstructure.Superstructure.ArmTargets;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class RobotContainer {
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  // private final Superstructure arm = Superstructure.getInstance();

  CommandPS4Controller driverGamepadPS4 = new CommandPS4Controller(0);
  CommandPS4Controller operatorGamepadPS4 = new CommandPS4Controller(1);
  CommandXboxController driverGamepadXbox = new CommandXboxController(0);
  CommandXboxController operatorGamepadXbox = new CommandXboxController(1);

  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  Command driveCommand;
  Command driveSim;

  BooleanPublisher alignmentPub;
  BooleanSubscriber alignmentSub;

  BooleanPublisher sniperPub;
  BooleanSubscriber sniperSub;

  BooleanPublisher robotPub;
  BooleanSubscriber robotSub;

  BooleanPublisher reverseIntakePub;
  BooleanSubscriber reverseIntakeSub;
  
  DoublePublisher angularVelocityPub;
  DoubleSubscriber angularVelocitySub;

  private SimulatedArena arena;

  StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
    .getStructArrayTopic("FieldElements/Alage", Pose3d.struct)
    .publish();

  StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
    .getStructArrayTopic("FieldElements/Coral", Pose3d.struct)
    .publish();

  public RobotContainer() {
    configureBindings();

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
   
    reverseIntakePub = table.getBooleanTopic("reverseIntake").publish();
    reverseIntakePub.set(false);
    reverseIntakeSub = table.getBooleanTopic("reverseIntake").subscribe(false);

    if (DriverStation.getJoystickIsXbox(0)) {
      driveCommand = drivetrain.driveCommand(
        () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepadXbox.getLeftY() * 0.25 : -driverGamepadXbox.getLeftY(),
        OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepadXbox.getLeftX() * 0.25 : -driverGamepadXbox.getLeftX(),
        OperatorConstants.LEFT_X_DEADBAND),
        () -> alignmentSub.get() ? angularVelocitySub.get() : sniperSub.get() ? -driverGamepadXbox.getRightX() * 0.75 : -driverGamepadXbox.getRightX(),
        () -> robotSub.get());
    } else {
      driveCommand = drivetrain.driveCommand(
        () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepadPS4.getLeftY() * 0.25 : -driverGamepadPS4.getLeftY(),
        OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepadPS4.getLeftX() * 0.25 : -driverGamepadPS4.getLeftX(),
        OperatorConstants.LEFT_X_DEADBAND),
        () -> alignmentSub.get() ? angularVelocitySub.get() : sniperSub.get() ? -driverGamepadPS4.getRightX() * 0.75 : -driverGamepadPS4.getRightX(),
        () -> robotSub.get());
    }
        
    drivetrain.setDefaultCommand(driveCommand);
    // arm.setDefaultCommand(new MoveArm());

    NamedCommands.registerCommand("creep", new Creep(0));
    NamedCommands.registerCommand("creepAmp", new Creep(1));
    NamedCommands.registerCommand("angleSource", new AutoAngle(4, true));
    
    autoChooser.setDefaultOption("Shoot Creep", "Shoot Creep");
    SmartDashboard.putData("Auto", autoChooser);
  }

  private void configureBindings() {
    if (DriverStation.getJoystickIsXbox(0)) {
      driverGamepadXbox.a().whileTrue(new AutoAngle(1, false));
      driverGamepadXbox.b().whileTrue(new AutoAngle(2, false));
      driverGamepadXbox.x().whileTrue(new AutoAngle(3, false));
      driverGamepadXbox.y().whileTrue(new AutoAngle(4, false));
      driverGamepadXbox.leftBumper().whileTrue(new LimelightTracking());
      driverGamepadXbox.leftTrigger().whileTrue(new SniperMode());
      driverGamepadXbox.rightTrigger().whileTrue(new RobotRelative());
      driverGamepadXbox.start().onTrue(new InstantCommand(drivetrain::zeroGyro));
    } else {
      driverGamepadPS4.cross().whileTrue(new AutoAngle(1, false));
      driverGamepadPS4.circle().whileTrue(new AutoAngle(2, false));
      driverGamepadPS4.square().whileTrue(new AutoAngle(3, false));
      driverGamepadPS4.triangle().whileTrue(new AutoAngle(4, false));
      driverGamepadPS4.L1().whileTrue(new LimelightTracking());
      driverGamepadPS4.L2().whileTrue(new SniperMode());
      driverGamepadPS4.R2().whileTrue(new RobotRelative());
      driverGamepadPS4.options().onTrue(new InstantCommand(drivetrain::zeroGyro));
    }
    
    if (DriverStation.getJoystickIsXbox(1)) {
      operatorGamepadXbox.a().onTrue(new SetArmTarget(ArmTargets.L1));
      operatorGamepadXbox.b().onTrue(new SetArmTarget(ArmTargets.L2));
      operatorGamepadXbox.x().onTrue(new SetArmTarget(ArmTargets.L3));
      operatorGamepadXbox.y().onTrue(new SetArmTarget(ArmTargets.L4));
      operatorGamepadXbox.leftBumper().onTrue(new SetArmTarget(ArmTargets.HUMAN_PLAYER));
      operatorGamepadXbox.rightBumper().onTrue(new SetArmTarget(ArmTargets.PROCESSOR));
      operatorGamepadXbox.povRight().onTrue(new SetArmTarget(ArmTargets.ALGAE_REMOVAL));
      operatorGamepadXbox.leftTrigger().whileTrue(new RunIntake());
      operatorGamepadXbox.rightTrigger().whileTrue(new ReverseIntake());
      operatorGamepadXbox.povUp().whileTrue(new Climb(true));
      operatorGamepadXbox.povDown().whileTrue(new Climb(false));
    } else {
      operatorGamepadPS4.cross().onTrue(new SetArmTarget(ArmTargets.L1));
      operatorGamepadPS4.circle().onTrue(new SetArmTarget(ArmTargets.L2));
      operatorGamepadPS4.square().onTrue(new SetArmTarget(ArmTargets.L3));
      operatorGamepadPS4.triangle().onTrue(new SetArmTarget(ArmTargets.L4));
      operatorGamepadPS4.povRight().onTrue(new SetArmTarget(ArmTargets.ALGAE_REMOVAL));
      operatorGamepadPS4.L2().whileTrue(new RunIntake());
      operatorGamepadPS4.R2().whileTrue(new ReverseIntake());
      operatorGamepadPS4.povUp().whileTrue(new Climb(true));
      operatorGamepadPS4.povDown().whileTrue(new Climb(false));
    }
  }

  public Command getAutonomousCommand() {
    return drivetrain.getAutonomousCommand(autoChooser.getSelected());
  }

  public void setMotorBrake(boolean brake) {
    drivetrain.setMotorBrake(brake);
  }

  public double getOpLeftStickY() {
    return DriverStation.getJoystickIsXbox(1) ? operatorGamepadXbox.getLeftY() : operatorGamepadPS4.getLeftY();
  }

  public double getOpRightStickY() {
    return DriverStation.getJoystickIsXbox(1) ? operatorGamepadXbox.getRightY() : operatorGamepadPS4.getRightY();
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
