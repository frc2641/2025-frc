package frc.team2641.robot2025;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team2641.robot2025.Constants.OperatorConstants;
import frc.team2641.robot2025.commands.*;
import frc.team2641.robot2025.commands.auto.*;
import frc.team2641.robot2025.commands.elevatorMoves.*;
import frc.team2641.robot2025.commands.shifts.*;
import frc.team2641.robot2025.commands.spinIntake.*;
import frc.team2641.robot2025.subsystems.Drivetrain;


public class RobotContainer {
  private final Drivetrain drivetrain = Drivetrain.getInstance();

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

  public RobotContainer() {
    configureBindings();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");

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
        () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepad.getLeftY() * 0.25 : -driverGamepad.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepad.getLeftX() * 0.25 : -driverGamepad.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> alignmentSub.get() ? angularVelocitySub.get() : sniperSub.get() ? -driverGamepad.getRightX() * 0.75 : -driverGamepad.getRightX(),
        () -> robotSub.get());
    
    drivetrain.setDefaultCommand(driveCommand);

    NamedCommands.registerCommand("creep", new Creep(0));
    NamedCommands.registerCommand("creepAmp", new Creep(1));
    NamedCommands.registerCommand("angleSource", new AutoAngle(4, true));
    
    autoChooser.setDefaultOption("Shoot Creep", "Shoot Creep");
    SmartDashboard.putData("Auto", autoChooser);
  }

  private void configureBindings() {
    driverGamepad.a().whileTrue(new AutoAngle(1, false));
    driverGamepad.b().whileTrue(new AutoAngle(2, false));
    driverGamepad.x().whileTrue(new AutoAngle(3, false));
    driverGamepad.y().whileTrue(new AutoAngle(4, false));
    driverGamepad.leftBumper().whileTrue(new LimelightTracking());
    driverGamepad.rightBumper().whileTrue(new Climb()); /* toggles climber state... maybe */
    driverGamepad.leftTrigger().whileTrue(new SniperMode());
    driverGamepad.rightTrigger().whileTrue(new RobotRelative());
    driverGamepad.start().onTrue(new InstantCommand(drivetrain::zeroGyro));
    
    operatorGamepad.a().onTrue(new MoveElevator1()); /*L1 */
    operatorGamepad.b().onTrue(new MoveElevator2()); /*L2 */
    operatorGamepad.x().onTrue(new MoveElevator3()); /*L3 */
    operatorGamepad.y().onTrue(new MoveElevator4()); /*L4 */
    operatorGamepad.leftBumper().onTrue(new MoveElevator5()); /*Human Player */
    operatorGamepad.rightBumper().onTrue(new MoveElevator6()); /*Processor */
    operatorGamepad.leftTrigger().whileTrue(new In());
    operatorGamepad.rightTrigger().whileTrue(new Out());
    
    operatorGamepad.povUp().whileTrue(new Up());
    operatorGamepad.povDown().whileTrue(new Down());
    

  }

  public Command getAutonomousCommand() {
    return drivetrain.getAutonomousCommand(autoChooser.getSelected());
  }

  public void setMotorBrake(boolean brake) {
    drivetrain.setMotorBrake(brake);
  }
}
