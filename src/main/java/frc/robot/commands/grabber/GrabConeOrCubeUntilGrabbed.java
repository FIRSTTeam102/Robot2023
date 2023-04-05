package frc.robot.commands.grabber;

import static frc.robot.constants.GrabberConstants.*;

import frc.robot.io.VisionIO.GamePieceString;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.littletonrobotics.junction.Logger;

public class GrabConeOrCubeUntilGrabbed extends CommandBase {
	Grabber grabber;
	Vision vision;

	String lastPiece;

	public GrabConeOrCubeUntilGrabbed(Grabber grabber, Vision vision) {
		this.grabber = grabber;
		this.vision = vision;
		addRequirements(grabber);
	}

	@Override
	public void initialize() {
		lastPiece = "";
	}

	@Override
	public void execute() {
		if (lastPiece.equals("") || !vision.inputs.gamePieceVisionTargetObjectClass.equals(""))
			lastPiece = vision.inputs.gamePieceVisionTargetObjectClass;
		Logger.getInstance().recordOutput("Vision/lastPiece", lastPiece);
		Logger.getInstance().recordOutput("Vision/true", lastPiece.equals(GamePieceString.Cone));

		grabber.move(lastPiece.equals(GamePieceString.Cone)
			? coneGrabSpeed
			: cubeGrabSpeed);
	}

	@Override
	public boolean isFinished() {
		return grabber.hasGrabbed(grabbedTicks);
	}

	@Override
	public void end(boolean interrupted) {
		grabber.hold();
	}
}
