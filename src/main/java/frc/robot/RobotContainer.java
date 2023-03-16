package frc.robot;

import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CameraConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.Constants.ShuffleboardConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.GrabberConstants;
import frc.robot.constants.ScoringPosition;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon2;
import frc.robot.io.GyroIOSim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

import frc.robot.commands.Autos;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.arm.ManualArmControl;
import frc.robot.commands.elevator.ManualElevatorControl;
import frc.robot.commands.elevator.MoveElevatorBy;
import frc.robot.commands.grabber.GrabGrabber;
import frc.robot.commands.grabber.ReleaseGrabber;
import frc.robot.commands.grabber.StopGrabber;
import frc.robot.commands.scoring.SetScoringPosition;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.XStance;
import frc.robot.commands.vision.AprilTagVision;
import frc.robot.commands.vision.GamePieceVision;
import frc.robot.commands.vision.RetroreflectiveVision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	private static RobotContainer instance = null;

	public static RobotContainer getInstance() {
		if (instance == null)
			instance = new RobotContainer();
		return instance;
	}

	public final CommandXboxController driverController = new CommandXboxController(
		OperatorConstants.driverControllerPort);
	public final CommandGenericHID operatorConsole = new CommandGenericHID(
		OperatorConstants.operatorConsolePort);
	public final CommandJoystick operatorJoystick = new CommandJoystick(
		OperatorConstants.operatorJoystickPort);

	public final GyroIO gyro = Robot.isReal()
		? new GyroIOPigeon2(Constants.pigeonId)
		: new GyroIOSim();

	/* subsystems */
	public final Vision vision = new Vision();
	public final Swerve swerve = new Swerve(gyro, vision);
	public final Arm arm = new Arm();
	public final Elevator elevator = new Elevator();
	public final Grabber grabber = new Grabber();

	private LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("auto routine");
	// private GenericEntry autoDelay;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);

		configureBindings();

		// setup autos
		// FW = field wall, LZ = loading zone barrier
		autoChooser.addDefaultOption("nothing", new InstantCommand());
		autoChooser.addOption("2 piece (FW)", Autos.twoPieceFW(this));
		autoChooser.addOption("2 piece & balance (FW)", Autos.twoPieceFWChargeStation(this));
		autoChooser.addOption("2 piece (LZ)", Autos.twoPieceLZ(this));
		autoChooser.addOption("2 piece & balance (LZ)", Autos.twoPieceLZChargeStation(this));
		autoChooser.addOption("1 piece (LZ)", Autos.loadingZone(this));
		autoChooser.addOption("1 piece (FW)", Autos.fieldWallOnePiece(this));
		autoChooser.addOption("backup & balance (co-op)", Autos.coopBackup(this));
		autoChooser.addOption("just score", Autos.justScore(this));

		// for testing
		autoChooser.addOption("[testing] drive characterization",
			new FeedForwardCharacterization(swerve, true, swerve::runCharacterization, swerve::getCharacterizationVelocity));

		var driveTab = Shuffleboard.getTab(ShuffleboardConstants.driveTab);
		driveTab.add("alerts", Alert.getAlertsSendable())
			.withSize(2, 4).withPosition(15, 0);
		driveTab.add("auto routine", autoChooser.getSendableChooser())
			.withSize(4, 1).withPosition(0, 6);
		// autoDelay = driveTab.add("auto delay", 0.0)
		// .withSize(2, 1).withPosition(0, 7)
		// .getEntry();
		configureCameras();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController Xbox} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
	 */
	private void configureBindings() {
		/* driver */
		var teleopSwerve = new TeleopSwerve(
			() -> -driverController.getLeftY(),
			() -> -driverController.getLeftX(),
			() -> -driverController.getRightX(),
			driverController.getHID()::getLeftBumper, // override speed
			() -> driverController.getLeftTriggerAxis() > OperatorConstants.boolTriggerThreshold, // preceise mode
			swerve, arm, elevator);
		swerve.setDefaultCommand(teleopSwerve);

		driverController.rightTrigger(OperatorConstants.boolTriggerThreshold)
			.whileTrue(teleopSwerve.holdToggleFieldRelative());

		driverController.a().onTrue(teleopSwerve.toggleFieldRelative());
		driverController.x().whileTrue(new XStance(swerve));

		var zeroYaw = teleopSwerve.new ZeroYaw();
		driverController.y().onTrue(zeroYaw);
		new Trigger(RobotController::getUserButton).onTrue(zeroYaw);

		driverController.povUp().whileTrue(new ChargeStationBalance(swerve));

		/*
		 * operator console
		 * G1 G2 G3 B4
		 * G5 Y6 Y7 B8
		 * R9 Y10 Y11 B12
		 * R13 R14 R15 B16
		 */

		// swerve to grid or double substation. swerve/arm/elevator game piece (green)
		operatorConsole.button(1) // left grid or left double substation
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.BlueRedGridDoublesubstationLeft, vision, swerve));
		operatorConsole.button(2) // middle grid
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.BlueRedGridMiddle, vision, swerve));
		operatorConsole.button(3) // right grid or right double substation
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.BlueRedGridDoublesubstationRight, vision, swerve));

		// swerve or/and arm/elevator to grid or double substation (blue + orange)
		operatorConsole.button(4) // double substation
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.DoubleSubstation));
		operatorConsole.button(7) // high cone
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.HighCone))
			.whileTrue(new RetroreflectiveVision(RetroreflectiveVision.Routine.BlueRedGridLeftRight, vision, swerve));
		operatorConsole.button(8) // high cube
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.HighCube));
		operatorConsole.button(11) // mid cone
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.MidCone))
			.whileTrue(new RetroreflectiveVision(RetroreflectiveVision.Routine.BlueRedGridLeftRight, vision, swerve));
		operatorConsole.button(12) // mid cube
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.MidCube));
		operatorConsole.button(15) // all in
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.AllIn));
		operatorConsole.button(16) // ground
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.Ground));
		operatorConsole.button(10)
			.onTrue(new SetScoringPosition(elevator, arm, ScoringPosition.SingleSubstationCube));

		operatorConsole.button(5) // aim at game piece
			.whileTrue(new GamePieceVision(GamePieceVision.Routine.GamepieceGround, vision, swerve, elevator, arm, grabber));

		operatorConsole.button(14)
			.whileTrue(Commands.run(elevator::killMotor, elevator));
		// .whileTrue(arm.tempDisableLimits);

		// human player request
		operatorConsole.button(9).onTrue(Lights.requestGamePiece(Lights.Status.Cone));
		operatorConsole.button(13).onTrue(Lights.requestGamePiece(Lights.Status.Cube));

		/*
		 * operator flight stick
		 */
		arm.setDefaultCommand(new ManualArmControl(arm, () -> -operatorJoystick.getX()));
		elevator.setDefaultCommand(new ManualElevatorControl(elevator, () -> operatorJoystick.getY()));
		// todo: what happens when both pressed?
		operatorJoystick.trigger()
			.whileTrue(new GrabGrabber(grabber, GrabberConstants.cubeGrabSpeed));
		operatorJoystick.button(5)
			.whileTrue(new GrabGrabber(grabber, GrabberConstants.coneGrabSpeed));
		operatorJoystick.button(2)
			.whileTrue(new ReleaseGrabber(grabber));
		operatorJoystick.button(3)
			.onTrue(new MoveElevatorBy(elevator, ElevatorConstants.coneMoveDownHeight_m)
				.andThen(new ReleaseGrabber(grabber)));
		operatorJoystick.button(4)
			.onTrue(new StopGrabber(grabber));
		// operatorJoystick.button(5)
		// .onTrue(new GrabGrabberUntilGrabbed(grabber));
	}

	@SuppressWarnings("unused")
	private void configureCameras() {
		if (!Robot.isReal())
			return;

		// camera
		try {
			var camera = CameraServer.startAutomaticCapture("arm", 0);
			camera.setConnectVerbose(0);
			// camera.setFPS(CameraConstants.fps);
			// camera.setResolution(CameraConstants.width, CameraConstants.height);
			// camera.setPixelFormat(PixelFormat.kGray);
			if (CameraConstants.exposure >= 0)
				camera.setExposureManual(CameraConstants.exposure);
			else
				camera.setExposureAuto();
			// camera.setBrightness(0);

			// camera server is evil
			// var cameraServer = CameraServer.addSwitchedCamera("camera");
			// var cameraServer = CameraServer.startAutomaticCapture(camera);
			// var cameraServer = (MjpegServer) CameraServer.getServer();
			// cameraServer.setFPS(CameraConstants.fps);
			// cameraServer.setResolution(CameraConstants.width, CameraConstants.height);
			// cameraServer.setCompression(CameraConstants.compression);

			Shuffleboard.getTab(ShuffleboardConstants.driveTab)
				.add("camera", SendableCameraWrapper.wrap(camera))
				// .addCamera("camera", "arm", cameraServ
				.withWidget(BuiltInWidgets.kCameraStream)
				.withSize(8, 6)
				.withPosition(0, 0);
		} catch (edu.wpi.first.cscore.VideoException e) {
			DriverStation.reportError("Failed to get camera: " + e.toString(), e.getStackTrace());
		}

		// if (Robot.isReal()) {
		// var limelightCamera = new HttpCamera("limelightStream", "http://limelight.local:5800/stream.mjpg",
		// HttpCameraKind.kMJPGStreamer);
		// Shuffleboard.getTab(ShuffleboardConstants.driveTab)
		// .add("limelight", limelightCamera)
		// .withProperties(Map.of("show crosshair", false, "show controls", false))
		// .withWidget(BuiltInWidgets.kCameraStream)
		// .withSize(7, 6)
		// .withPosition(8, 0);
		// }
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// if (autoDelay.getDouble(0) > 0)
		// return Commands.waitSeconds(autoDelay.getDouble(0)).andThen(autoChooser.get());
		return autoChooser.get();
	}

	Alert driverControllerAlert = new Alert("driver controller not connected properly", AlertType.Error);
	Alert operatorConsoleAlert = new Alert("operator console not connected properly", AlertType.Error);
	Alert operatorJoystickAlert = new Alert("operator joystick not connected properly", AlertType.Error);

	public void updateOIAlert() {
		if (!driverController.getHID().isConnected()
			|| driverController.getHID().getName().indexOf("Xbox") < 0)
			driverControllerAlert.set(true);
		else
			driverControllerAlert.set(false);

		if (!operatorConsole.getHID().isConnected()
			|| operatorConsole.getHID().getName().indexOf("CyController") < 0)
			operatorConsoleAlert.set(true);
		else
			operatorConsoleAlert.set(false);

		if (!operatorJoystick.getHID().isConnected()
			|| operatorJoystick.getHID().getName().indexOf("Logitech") < 0)
			operatorJoystickAlert.set(true);
		else
			operatorJoystickAlert.set(false);
	}
}