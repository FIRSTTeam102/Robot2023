package frc.robot.commands.swerve;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * sets the swerve modules in the x-stance orientation.
 * the wheels are aligned to make an 'X',
 * which makes it more difficult for other robots to push us.
 */
public class XStance extends CommandBase {
	private Swerve swerve;

	public XStance(Swerve swerve) {
		addRequirements(swerve);
		this.swerve = swerve;
	}

	@Override
	public void initialize() {
		swerve.setModuleStates(swerve.getXStanceStates(), true, true);
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
