package frc.team2641.robot2025;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team2641.robot2025.Constants.AutoAngleNum;
import frc.team2641.robot2025.Constants.ELEVNUM;
import frc.team2641.robot2025.Constants.OperatorConstants;
import frc.team2641.robot2025.Constants.PIPELINE;
import frc.team2641.robot2025.commands.auto.*;
import frc.team2641.robot2025.commands.climbing.Climb;
import frc.team2641.robot2025.commands.climbing.Wrap;
import frc.team2641.robot2025.commands.elevator.MoveElevator;
import frc.team2641.robot2025.commands.elevator.SetElevator;
import frc.team2641.robot2025.commands.intake.RunIntake;
import frc.team2641.robot2025.commands.intake.RunOuttake;
import frc.team2641.robot2025.commands.intake.SuperSpin;
import frc.team2641.robot2025.commands.shifts.*;
import frc.team2641.robot2025.subsystems.*;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class RobotContainer {
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Elevator elevator = Elevator.getInstance();

  private final CommandXboxController driverGamepad = new CommandXboxController(0);
  private final CommandXboxController operatorGamepad = new CommandXboxController(1);

  private Command driveCommand;
  
  private BooleanPublisher alignmentPub;
  public BooleanSubscriber alignmentSub;
  
  private BooleanPublisher sniperPub;
  public BooleanSubscriber sniperSub;
  
  private BooleanPublisher robotPub;
  public BooleanSubscriber robotSub;
  
  private DoublePublisher angularVelocityPub;
  public DoubleSubscriber angularVelocitySub;
  private NetworkTable table;

  public RobotContainer() {
    driverGamepad.leftTrigger().whileTrue(new SniperMode());
    driverGamepad.rightTrigger().whileTrue(new RobotRelative());
    driverGamepad.start().onTrue(new InstantCommand(drivetrain::zeroGyro));
    driverGamepad.povUp().whileTrue(new Climb(true));
    driverGamepad.povDown().whileTrue(new Climb(false));
    driverGamepad.povLeft().whileTrue(new Wrap(true));
    driverGamepad.povRight().whileTrue(new Wrap(false));

    driverGamepad.x().whileTrue(new AutoAngle(AutoAngleNum.LEFT_HP, false));
    driverGamepad.y().whileTrue(new AutoAngle(AutoAngleNum.RIGHT_HP, false));
    driverGamepad.a().whileTrue(new LimelightTracking(PIPELINE.left));
    driverGamepad.b().whileTrue(new LimelightTracking(PIPELINE.right));

    operatorGamepad.leftTrigger().whileTrue(new RunIntake());
    operatorGamepad.rightTrigger().whileTrue(new RunOuttake());
    operatorGamepad.a().onTrue(new SetElevator(ELEVNUM.L1));
    operatorGamepad.b().onTrue(new SetElevator(ELEVNUM.L2));
    operatorGamepad.x().onTrue(new SetElevator(ELEVNUM.L3));
    operatorGamepad.y().onTrue(new SetElevator(ELEVNUM.L4));
    operatorGamepad.start().onTrue(new SetElevator(ELEVNUM.HP));
    operatorGamepad.back().onTrue(new SetElevator(0));

    operatorGamepad.rightBumper().whileTrue(new SuperSpin());
    
    table = NetworkTableInstance.getDefault().getTable("state");
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

    // with SRL
    driveCommand = drivetrain.driveCommand(
      () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepad.getLeftY() * Constants.DriveConstants.SNIPER_MODE : -driverGamepad.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepad.getLeftX() * Constants.DriveConstants.SNIPER_MODE : -driverGamepad.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> alignmentSub.get() ? angularVelocitySub.get() : sniperSub.get() ? -driverGamepad.getRightX() * 0.75 : -driverGamepad.getRightX(),
      () -> robotSub.get());
    // w/o SRL
    // driveCommand = drivetrain.driveCommand(
    //   () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepad.getLeftY() * Constants.DriveConstants.SNIPER_MODE : -driverGamepad.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //   () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepad.getLeftX() * Constants.DriveConstants.SNIPER_MODE : -driverGamepad.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //   () -> alignmentSub.get() ? angularVelocitySub.get() : sniperSub.get() ? -driverGamepad.getRightX() * Constants.DriveConstants.ANGLE_SNIPER_MODE : -driverGamepad.getRightX(),
    //   () -> robotSub.get());

    drivetrain.setDefaultCommand(driveCommand);
    elevator.setDefaultCommand(new MoveElevator());

    Autos.init();

    updateAlerts();
  }

  public Command getAutonomousCommand() {
    switch (Autos.getMode()) {
      case SIMPLE:
        return drivetrain.getAutonomousCommand(Autos.getSimpleAuto());
      case BYO:
        return Autos.getAutoCommand();
      case CREEP:
        return new Creep();
      default:
        return Commands.none();
    }
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

  public CommandXboxController getDriverGamepad() {
    return driverGamepad;
  }

  public CommandXboxController getOperatorGamepad() {
    return operatorGamepad;
  }

  public void updateAlerts() {
    Alerts.MissingOperatorGamepad.set(!operatorGamepad.isConnected());
    Alerts.MissingDriverGamepad.set(!driverGamepad.isConnected());
  }

  public NetworkTable getTable() {
    return table;
  }
}
