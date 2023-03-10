package frc.robot.constants;

public final class VisionConstants {
	/* AprilTagVision */
	public static final double poseError_m = 1; // comparing visionPose to pose
	public static final double botpose_fieldOffsetX_m = 0.18; // realife offset to pathplanner app
	public static final double maxZDistanceAprilTag_m = 1.8934684 /* grid to charge station */
		- 0.5334; /* limelight to front bumper */

	/* RetroreflectiveVision */
	public static final double retroreflectiveTranslateKp = 0.2;
	public static final double retroreflectiveTranslateKd = 0.025;
	public static final double crosshairTargetBoundTranslateX_rad = Math.toRadians(1.5);

	/* ObjectDetectionVision */
	public static final double gamePieceRotateKp = 0;
	public static final double gamePieceRotateKd = 0;
	public static final double crosshairGamePieceBoundRotateX_rad = 0.035;
};
