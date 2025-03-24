package frc.team2641.robot2025.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import java.util.Optional;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import frc.team2641.robot2025.Robot;
// import frc.team2641.robot2025.subsystems.elevator.ElevatorSimulation;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class IntakeIOSim implements IntakeIO {
  private final IntakeSimulation coralIntakeSim;
  // private final ElevatorSimulation elevSim;

  private Optional<SwerveDriveSimulation> driveSim;
  private BooleanSubscriber spinSub;

  private static IntakeIOSim instance;
  public static IntakeIOSim getInstance() {
    if (instance == null) instance = new IntakeIOSim();
    return instance;
  }
  
  public IntakeIOSim() {
    driveSim = Drivetrain.getInstance().getSwerveDrive().getMapleSimDrive();
    // elevSim = ElevatorSimulation.getInstance();
    

    // Here, create the intake simulation with respect to the intake on your real robot

    // Here, create the intake simulation with respect to the intake on your real robot
    this.coralIntakeSim = IntakeSimulation.OverTheBumperIntake(
      "Coral",
      driveSim.get(),
      // Width of the intake
      Meters.of(0.7),
      // The extension length of the intake beyond the robot's frame (when activated)
      Meters.of(0.2),
      // The intake is mounted on the back side of the chassis
      IntakeSimulation.IntakeSide.FRONT,
      1); 

    coralIntakeSim.setGamePiecesCount(1);
  }

  
  public void intake() {
    coralIntakeSim.startIntake();
  }

  
  public void stop() {
    coralIntakeSim.stopIntake();
  }

  public void spin() {
    if (spinSub.get()) outtake(); 
    else intake();
  }

  private void outtake(){
    if (coralIntakeSim.getGamePiecesAmount() > 0) {
      Robot.getArena()
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
          // Meters.of(1.28),
          // Meters.of(elevSim.getPosition()),
          Meters.of(0),
          // The initial speed of the coral
          MetersPerSecond.of(2),
          // The coral is ejected at a 35-degree slope
          Degrees.of(-35)
        )
      );
      coralIntakeSim.setGamePiecesCount(0);
    } 
  }

  public void setHeight(){
    
  }

  @Override
  public void firstSpin() {
    intake();
  }

  @Override
  public void secondSpin() {
    outtake();
  }

  public void superSpin(){
    if (coralIntakeSim.getGamePiecesAmount() > 0) {
      Robot.getArena()
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
          // Meters.of(1.28),
          Meters.of(0),
          // Meters.of(elevSim.getPosition()),
          // The initial speed of the coral
          MetersPerSecond.of(-5),
          // The coral is ejected at a 35-degree slope
          Degrees.of(-35)
        )
      );
      coralIntakeSim.setGamePiecesCount(0);
    } 

  }
}