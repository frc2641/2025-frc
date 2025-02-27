package frc.team2641.robot2025.subsystems.superstructure.wrist;

public interface WristIO {
    void setPosition(double pos);
    void set(double speed);
    double getPosition();
}