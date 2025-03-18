package frc.team2641.robot2025.subsystems.superstructure.wrist;

public interface WristIO {
	void goTo(double pos);
	double getPosition();
	double getSetpoint();
	void stop();
	void resetEncoder();
}