package frc.team2641.robot2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ironmaple.simulation.SimulatedArena;

public class Telemetry {
	private static Telemetry instance = null;

	private SimulatedArena arena;

	public static Telemetry getInstance() {
		if (instance == null) instance = new Telemetry();
		return instance;
	}

	public final Field2d field2d;

	StructArrayPublisher<Pose3d> notePoses = NetworkTableInstance.getDefault()
		.getStructArrayTopic("MyPoseArray", Pose3d.struct)
		.publish();

	public Telemetry() {
		this.field2d = new Field2d();
		SmartDashboard.putData("field", field2d);
		if (RobotBase.isSimulation()) arena = Robot.getArena();
	}

	public Field2d getFieldWidget() {
		return field2d;
	}

	public void feedSimulationRobotPose(Pose2d simulationRobotPose) {
		field2d.setRobotPose(simulationRobotPose);
	}

	public void feedOdometryPose(Pose2d odometryPose) {
		if (RobotBase.isReal()) field2d.setRobotPose(odometryPose);
		else field2d.getObject("Odometry").setPose(odometryPose);
	}

	public void publishSimulationNotePoses() {
		notePoses.accept(arena.getGamePiecesArrayByType("Note"));
	}
}