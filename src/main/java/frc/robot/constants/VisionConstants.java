package frc.robot.constants;

public final class VisionConstants {
	/* AprilTagVision */
	public static final double poseError_m = 1; // comparing visionPose to pose
	public static final double botpose_fieldOffsetX_m = 0.18; // realife offset to pathplanner app
	public static final double botpose_fieldBlueCommunityGeoFenceX_m = 2.84; /* blue left to charge station distance */
	public static final double botpose_fieldRedCommunityGeoFenceX_m = 13.68; /* red right to charge station distance */

	/* RetroreflectiveVision */
	public static final double retroreflectiveTranslateKp = 2.0;
	public static final double retroreflectiveTranslateKd = 0.25;
	public static final double crosshairTargetBoundTranslateX_rad = Math.toRadians(1.5);

	/* ObjectDetectionVision */
	public static final double gamePieceRotateKp = 0;
	public static final double gamePieceRotateKd = 0;
	public static final double crosshairGamePieceBoundRotateX_rad = 0.035;
};
