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
	private double rotateErrorSum;
	public double rotateErrorIntegral;
	private double rotateErrorCount;
	private double rotateErrorDifference;
	public double rotateErrorDerivative;

	private double translateErrorSum;
	public double translateErrorIntegral;
	private double translateErrorCount;
	private double translateErrorDifference;
	public double translateErrorDerivative;

	Timer pipelineSwitchTimer = new Timer();
	LinkedList<Double> rotateErrorTotalHistory = new LinkedList<Double>();
	LinkedList<Double> rotateErrorLastTwoHistory = new LinkedList<Double>();
	LinkedList<Double> translateErrorTotalHistory = new LinkedList<Double>();
	LinkedList<Double> translateErrorLastTwoHistory = new LinkedList<Double>();

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
		rotateErrorTotalHistory.add(inputs.crosshairToTargetErrorX_rad);
		rotateErrorSum += rotateErrorTotalHistory.getLast();
		rotateErrorIntegral = rotateErrorSum * VisionConstants.periodicTime_s;

		// Getting difference between crosshairToTargetoffsetX_rad within the last 0.02s
		rotateErrorLastTwoHistory.add(inputs.crosshairToTargetErrorX_rad);
		rotateErrorCount += 1;
		if (rotateErrorCount == 2) {
			rotateErrorDifference = rotateErrorLastTwoHistory.getLast() - rotateErrorLastTwoHistory.getFirst();
			rotateErrorDerivative = rotateErrorDifference / VisionConstants.periodicTime_s;
			rotateErrorLastTwoHistory.removeFirst();
			rotateErrorCount -= 1;
		}

		// Getting total botpose_targetspaceTranslationZ_m sum every 0.02s
		translateErrorTotalHistory.add(inputs.botpose_targetspaceRotationZ_rad);
		translateErrorSum += translateErrorTotalHistory.getLast();
		translateErrorIntegral = translateErrorSum * VisionConstants.periodicTime_s;

		// Getting difference between botpose_targetspaceTranslationZ_m within the last 0.02s
		translateErrorLastTwoHistory.add(inputs.botpose_targetspaceRotationZ_rad);
		translateErrorCount += 1;
		if (translateErrorCount == 2) {
			translateErrorDifference = translateErrorLastTwoHistory.getLast() - translateErrorLastTwoHistory.getFirst();
			translateErrorDerivative = translateErrorDifference / VisionConstants.periodicTime_s;
			translateErrorLastTwoHistory.removeFirst();
			translateErrorCount -= 1;
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
