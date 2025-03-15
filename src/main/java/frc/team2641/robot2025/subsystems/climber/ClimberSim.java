package frc.team2641.robot2025.subsystems.climber;

public class ClimberSim implements ClimberIO {
	private static ClimberSim instance;

	public static ClimberSim getInstance(){
		if (instance == null) instance = new ClimberSim();
		return instance;
	}


	@Override
	public void extend() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'extend'");
	}

	@Override
	public void retract() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'retract'");
	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'stop'");
	}
}