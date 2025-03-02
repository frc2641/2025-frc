package frc.team2641.robot2025.subsystems.swerve;

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

  public static Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  public Drivetrain() {
    this.swerveDrive = super.getSwerveDrive();
  }

  public double getDistanceToSpeaker() {
    int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    Pose3d speakerAprilTagPose = super.getAprilTagFieldLayout().getTagPose(allianceAprilTag).get();
    return getPose().getTranslation().getDistance(speakerAprilTagPose.toPose2d().getTranslation());
  }

  public Rotation2d getSpeakerYaw() {
    int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    Pose3d speakerAprilTagPose = super.getAprilTagFieldLayout().getTagPose(allianceAprilTag).get();
    Translation2d relativeTrl = speakerAprilTagPose.toPose2d().relativeTo(getPose()).getTranslation();
    return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(swerveDrive.getOdometryHeading());
  }

  public Command aimAtSpeaker(double tolerance) {
    SwerveController controller = swerveDrive.getSwerveController();
    return run(
      () -> {
        ChassisSpeeds speeds = ChassisSpeeds
          .fromFieldRelativeSpeeds(0, 0, controller.headingCalculate(getHeading().getRadians(), getSpeakerYaw().getRadians()), getHeading());
        drive(speeds);
      }).until(() -> Math.abs(getSpeakerYaw().minus(getHeading()).getDegrees()) < tolerance);
  }
}
