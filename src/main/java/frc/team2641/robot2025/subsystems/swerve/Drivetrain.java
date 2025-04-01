package frc.team2641.robot2025.subsystems.swerve;

import java.util.NoSuchElementException;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.SwerveController;
import swervelib.SwerveDrive;

public class Drivetrain extends SwerveBase {
  private static Drivetrain instance = null;
  private SwerveDrive swerveDrive;

  private final Pose3d[] reefTagPoses;

  public static Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  public Drivetrain() {
    super();
    int allianceAprilTag = 17;
    try {
      allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 17 : 6;
      
    } catch (NoSuchElementException e) {
      e.printStackTrace();
    }
    reefTagPoses = new Pose3d[6];
    for (int x = 0; x < 6; x++)
      {
        reefTagPoses[x] = super.getAprilTagFieldLayout().getTagPose(allianceAprilTag + x).get();
      }
    this.swerveDrive = super.getSwerveDrive();
  }

  public double getDistanceToReef() {
    int bestIndex = 0;
    double minDistance = getPose().getTranslation().getDistance(reefTagPoses[0].toPose2d().getTranslation());

    for (int i = 1; i < reefTagPoses.length; i++){
      double distance = getPose().getTranslation().getDistance(reefTagPoses[i].toPose2d().getTranslation());
    
      if (distance < minDistance)
        {
          bestIndex = i;
          minDistance = distance;
        }
    }

    return getPose().getTranslation().getDistance(reefTagPoses[bestIndex].toPose2d().getTranslation());
  }
  private int getClosestReefIndex() {
    int bestIndex = 0;
    double minDistance = getPose().getTranslation().getDistance(reefTagPoses[0].toPose2d().getTranslation());

    for (int i = 1; i < reefTagPoses.length; i++){
      double distance = getPose().getTranslation().getDistance(reefTagPoses[i].toPose2d().getTranslation());
    
      if (distance < minDistance)
        {
          bestIndex = i;
          minDistance = distance;
        }
    }

return bestIndex;
  }
/** @return returns the yaw of the closest side of the reef */
  public Rotation2d getReefYaw() {
    
    Translation2d relativeTrl = reefTagPoses[getClosestReefIndex()].toPose2d().relativeTo(getPose()).getTranslation();
    return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(swerveDrive.getOdometryHeading());
  }

  public Command aimAtSpeaker(double tolerance) {
    SwerveController controller = swerveDrive.getSwerveController();
    return run(
      () -> {
        ChassisSpeeds speeds = ChassisSpeeds
          .fromFieldRelativeSpeeds(0, 0, controller.headingCalculate(getHeading().getRadians(), getReefYaw().getRadians()), getHeading());
        drive(speeds);
      }).until(() -> Math.abs(getReefYaw().minus(getHeading()).getDegrees()) < tolerance);
  }
}
