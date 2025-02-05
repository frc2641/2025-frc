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
import frc.team2641.robot2025.commands.shifts.*;
import frc.team2641.robot2025.subsystems.Arm;
import frc.team2641.robot2025.subsystems.Drivetrain;
import frc.team2641.robot2025.subsystems.Arm.switcher;


public class RobotContainer {
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Arm arm = Arm.getInstance();

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

  BooleanPublisher spinPub;
  BooleanSubscriber spinSub;
  

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
   
    spinPub = table.getBooleanTopic("intakeSpinningOut").publish();
    spinPub.set(false);
    spinSub = table.getBooleanTopic("intakeSpinningOut").subscribe(false);

    driveCommand = drivetrain.driveCommand(
        () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepad.getLeftY() * 0.25 : -driverGamepad.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(sniperSub.get() ? -driverGamepad.getLeftX() * 0.25 : -driverGamepad.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> alignmentSub.get() ? angularVelocitySub.get() : sniperSub.get() ? -driverGamepad.getRightX() * 0.75 : -driverGamepad.getRightX(),
        () -> robotSub.get());
    
    drivetrain.setDefaultCommand(driveCommand);
    arm.setDefaultCommand(new MoveArm());

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
    driverGamepad.leftTrigger().whileTrue(new SniperMode());
    driverGamepad.rightTrigger().whileTrue(new RobotRelative());
    driverGamepad.start().onTrue(new InstantCommand(drivetrain::zeroGyro));
    
    operatorGamepad.a().onTrue(new SetArmPosition(switcher.L1)); /*L1 */
    operatorGamepad.b().onTrue(new SetArmPosition(switcher.L2)); /*L2 */
    operatorGamepad.x().onTrue(new SetArmPosition(switcher.L3)); /*L3 */
    operatorGamepad.y().onTrue(new SetArmPosition(switcher.L4)); /*L4 */
    operatorGamepad.leftBumper().onTrue(new SetArmPosition(switcher.HUMAN_PLAYER)); /*Human Player */
    operatorGamepad.rightBumper().onTrue(new SetArmPosition(switcher.PROCESSOR)); /*Processor */
    operatorGamepad.leftTrigger().whileTrue(new Spin()); /* intake/outtake */
    operatorGamepad.rightTrigger().whileTrue(new IntakeSpinningOut());/* shift key */

    operatorGamepad.povUp().whileTrue(new Climb(true));
    operatorGamepad.povDown().whileTrue(new Climb(false));
  }

  public Command getAutonomousCommand() {
    return drivetrain.getAutonomousCommand(autoChooser.getSelected());
  }

  public void setMotorBrake(boolean brake) {
    drivetrain.setMotorBrake(brake);
  }

  public double getOpLeftJoy() {
    return operatorGamepad.getLeftY();
  }

  public double getOpRightJoy(){
    return operatorGamepad.getRightY();
  }
}
