package frc.robot.commands.grabber;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseGrabber extends CommandBase {
	private Grabber grabber;
	private Timer timer = new Timer();
	private double runTime_s;
	private double speed;

	public CloseGrabber(Grabber grabber, double speed, double runTime_s) {
		this.grabber = grabber;
		this.speed = speed;
		this.runTime_s = runTime_s;
		addRequirements(grabber);
	}

	@Override
	public void initialize() {
		grabber.move(speed);
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
		return (grabber.currentLimitReached || timer.hasElapsed(runTime_s));
	}

}
