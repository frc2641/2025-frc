package frc.team2641.robot2025.subsystems.superstructure.wrist;

public class WristSim implements WristIO {
	private static WristSim instance;

	public static WristSim getInstance(){
		if (instance == null) instance = new WristSim();
		return instance;
	}

	@Override
	public void goTo(double pos) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
	}

	@Override
	public double getPosition() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
	}

	@Override
	public double getSetpoint() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'set'");
	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'set'");
	}
}