package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.io.VisionIOInputsAutoLogged;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import java.util.LinkedList;

public class Vision extends SubsystemBase {
	private VisionIO io = new VisionIO();
	public VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
	private double errorSum;
	public double errorIntegral;
	private double errorCount;
	private double errorDifference;
	public double errorDerivative;

	Timer pipelineSwitchTimer = new Timer();
	LinkedList<Double> errorTotalHistory = new LinkedList<Double>();
	LinkedList<Double> errorLastTwoHistory = new LinkedList<Double>();

	public Vision() {
		setPipeline(Pipeline.AprilTag);
		setPipeline(Pipeline.Retroreflective);
		setPipeline(Pipeline.ObjectDetection);
	}

	@Override
	public void periodic() {

		// Every 0.02s, updating networktable variables
		io.updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		// Every 0.02s, updating pose2d
		if (inputs.pipeline == Pipeline.AprilTag.value && isPipelineReady())
			RobotContainer.getInstance().swerve.addVisionMeasurement(
				new Pose2d(inputs.botpose_targetspaceTranslationX_m, inputs.botpose_targetspaceTranslationY_m,
					new Rotation2d(inputs.botpose_targetspaceRotationZ_rad)));

		// Getting total crosshairToTargetOffsetX_rad sum every 0.02s
		errorTotalHistory.add(inputs.crosshairToTargetErrorX_rad);
		errorSum += errorTotalHistory.getLast();
		errorIntegral = errorSum * VisionConstants.periodicTime_s;

		// Getting difference between crosshairToTargetoffsetX_rad within the last 0.02s
		errorLastTwoHistory.add(inputs.crosshairToTargetErrorX_rad);
		errorCount += 1;
		if (errorCount == 2) {
			errorDifference = errorLastTwoHistory.getLast() - errorLastTwoHistory.getFirst();
			errorDerivative = errorDifference / VisionConstants.periodicTime_s;
			errorLastTwoHistory.removeFirst();
			errorCount -= 1;
		}

	}

	// Creates set pipeline
	public void setPipeline(Pipeline pipeline) {
		pipelineSwitchTimer.reset();
		pipelineSwitchTimer.start();
		io.setPipeline(pipeline);
	}

	// Allows AprilTag commands to begin after pipeline switch time error
	public boolean isPipelineReady() {
		if (pipelineSwitchTimer.hasElapsed(0.5)) {
			pipelineSwitchTimer.stop();
			return true;
		}
		return false;
	}
}
