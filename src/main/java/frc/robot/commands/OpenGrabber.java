package frc.robot.commands;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OpenGrabber extends CommandBase {
	private Grabber grabber;
	private Timer timer = new Timer();
	private double runTime_s;

	/**
	 * @param time_s how long to close
	*/
	public OpenGrabber(Grabber grabber, double runTime_s) {
		this.grabber = grabber;
		this.runTime_s = runTime_s;
		addRequirements(grabber);
	}

	@Override
	public void initialize() {
		grabber.moveOutward();
		timer.reset();
		timer.start();
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		grabber.stop();
		timer.stop();
	}

	@Override
	public boolean isFinished() {
		return timer.hasElapsed(runTime_s);
	}
}
