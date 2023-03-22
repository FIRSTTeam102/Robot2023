package frc.robot.commands.swerve;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AngleOffsetCalibration extends CommandBase {
	private Swerve swerve;

	private double[] sampledAngles = new double[4]; // sum of angles
	private double[] sendableAngles = sampledAngles; // average angles
	private int samples = 0;

	public AngleOffsetCalibration(Swerve swerve) {
		this.swerve = swerve;

		var group = Shuffleboard.getTab("calibration").getLayout("swerve", BuiltInLayouts.kList).withSize(5, 3);
		group.add("1. start then stop command while aligning modules", this);
		group.addDoubleArray("2. update angles in code", () -> sendableAngles);
	}

	@Override
	public void initialize() {
		swerve.resetModuleOffsets();

		sampledAngles = new double[] {0, 0, 0, 0};
		sendableAngles = new double[] {0, 0, 0, 0};
		samples = 0;
	}

	@Override
	public void execute() {
		samples++;
		var states = swerve.getStates();
		for (int i = 0; i < 4; i++) {
			sampledAngles[i] += states[i].angle.getRadians();
			sendableAngles[i] = sampledAngles[i] / samples;
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
