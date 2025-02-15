package frc.team2641.robot2025.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class IntakeIOSim implements IntakeIO {
  private final IntakeSimulation intakeSimulation;
  private Optional<SwerveDriveSimulation> driveSim;
  private BooleanSubscriber spinSub;

  private static IntakeIOSim instance;
  public static IntakeIOSim getInstance() {
    if (instance == null)
      instance = new IntakeIOSim();
    return instance;
  }
  
  public IntakeIOSim() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    spinSub = table.getBooleanTopic("reverseIntake").subscribe(false);


    driveSim = Drivetrain.getInstance().getSwerveDrive().getMapleSimDrive();

    // Here, create the intake simulation with respect to the intake on your real robot
    this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
      // Specify the type of game pieces that the intake can collect
      "Coral",
      // Specify the drivetrain to which this intake is attached
      driveSim.get(),
      // Width of the intake
      Meters.of(0.7),
      // The extension length of the intake beyond the robot's frame (when activated)
      Meters.of(0.2),
      // The intake is mounted on the back side of the chassis
      IntakeSimulation.IntakeSide.FRONT,
      // The intake can hold up to 1 note
      1
    );

    intakeSimulation.setGamePiecesCount(0);

  }

  @Override
  public void intake() {
    intakeSimulation.startIntake();
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    intakeSimulation.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
  }

  @Override
  public void shoot() {
  }

  @Override
  public void spin() {
    if (spinSub.get()&&intakeSimulation.getGamePiecesAmount()>0) 
      {
        SimulatedArena.getInstance()
    .addGamePieceProjectile(new ReefscapeCoralOnFly(
        // Obtain robot position from drive simulation
        driveSim.get().getSimulatedDriveTrainPose().getTranslation(),
        // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
        new Translation2d(0.35, 0.35),
        // Obtain robot speed from drive simulation
        driveSim.get().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        // Obtain robot facing from drive simulation
        driveSim.get().getSimulatedDriveTrainPose().getRotation(),
        // The height at which the coral is ejected
        Meters.of(1.28),
        // The initial speed of the coral
        MetersPerSecond.of(2),
        // The coral is ejected at a 35-degree slope
        Degrees.of(-35)));

         intakeSimulation.setGamePiecesCount(1);
      }
    else 
      intake();
  }
    
  // @Override // Defined by IntakeIO
  // public void shoot() {
  //   // if there is a coral in the intake, it will be removed and return true; otherwise, returns false
  //   if (intakeSimulation.obtainGamePieceFromIntake())
  //     IntakeIOSim.launchCoral(); // notify the simulated flywheels to launch a coral
  // }
}