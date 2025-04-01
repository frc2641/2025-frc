package frc.team2641.robot2025;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team2641.robot2025.commands.shifts.RobotRelative;
import frc.team2641.robot2025.commands.shifts.SniperMode;
import frc.team2641.robot2025.subsystems.Elevator;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;
import java.io.File;
import java.io.IOException;
import swervelib.parser.SwerveParser;

public class Robot extends TimedRobot {
  private static Robot instance;
  private Command autoCommand;

  private Drivetrain drivetrain;
  private Elevator elevator;

  private static PowerDistribution pdh;
  public RobotContainer robotContainer;

  private CommandXboxController driverGamepad;

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);

    pdh = new PowerDistribution(21, ModuleType.kRev);
    SmartDashboard.putData("PDH", pdh);
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());

    robotContainer = new RobotContainer();

    drivetrain = Drivetrain.getInstance();
    elevator = Elevator.getInstance();

    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    for (int port = 5800; port <= 5807; port++) 
      PortForwarder.add(port, "10.26.41.25", port);

    if (isSimulation())
      DriverStation.silenceJoystickConnectionWarning(true);
    else
      Elastic.selectTab("Pre-Match");

    driverGamepad = robotContainer.getDriverGamepad();
    robotContainer.setMotorBrake(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Time", DriverStation.getMatchTime());
  }

  @Override
  public void disabledInit() {
    if (Robot.isReal()) Elastic.selectTab("Pre-Match");
  }

  @Override
  public void disabledPeriodic() {
    elevator.goTo(elevator.getPosition());
  }

  @Override
  public void autonomousInit() {
    if (Robot.isReal()) Elastic.selectTab("Autonomous");
    autoCommand = robotContainer.getAutonomousCommand();
    if (autoCommand != null) autoCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (Robot.isReal()) Elastic.selectTab("Teleoperated");
    if (autoCommand != null) autoCommand.cancel();
    if (driverGamepad.leftTrigger().getAsBoolean()) new SniperMode().schedule();
    if (driverGamepad.rightTrigger().getAsBoolean()) new RobotRelative().schedule();
  }

  @Override
  public void teleopPeriodic() {
    driverGamepad.setRumble(
      RumbleType.kBothRumble,
      robotContainer.sniperSub.get()
        ? rumble(false)
        : driverGamepad.getLeftX() > 0.4 && drivetrain.getSwerveDrive().getFieldVelocity().vyMetersPerSecond < 0.1 
          ? rumble(true)
          : driverGamepad.getLeftY() > 0.4 && drivetrain.getSwerveDrive().getFieldVelocity().vyMetersPerSecond < 0.1 
            ? rumble(true)
            : rumble(false) 
    );
    SmartDashboard.putBoolean("April Tag Visible", canSeeID());
  }

  private int rumble(boolean on){
    SmartDashboard.putBoolean("Rumble", on);
    return on ? 1 : 0;
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
  }

  public static PowerDistribution getPDH() {
    return pdh;
  }

  public static boolean canSeeID(){
    return (int)Limelight.getFiducialID("") != -1;
  }
}
