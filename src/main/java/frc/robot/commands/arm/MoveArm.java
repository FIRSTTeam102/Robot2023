package frc.robot.commands.arm;

import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArm extends CommandBase {
	private Arm arm;
	private double speed;

	/** fixme: SHOULD NOT BE USED OUTSIDE OF TESTING */
	public MoveArm(Arm arm, double speed) {
		this.arm = arm;
		this.speed = speed;
		addRequirements(arm);
	}

	@Override
	public void initialize() {
		arm.setSpeed(speed);
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		arm.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
