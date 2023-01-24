package frc.robot.commands;

import frc.robot.subsystems.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseClaw extends CommandBase {
	private Claw claw;

	/** Creates a new CloseClaw. 
	 * 
	*/
	public CloseClaw(Claw claw) {
		this.claw = claw;
		addRequirements(claw);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		claw.moveInward();
	}

	@Override
	public void end(boolean interrupted) {
		claw.stop();
	}

	@Override
	public boolean isFinished() {
		return claw.isClosed();
	}
}
