package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	/**
	 * Subsystems
	 */
	private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	private final Swerve swerveSubsystem = new Swerve();

	private final CommandXboxController driverController = new CommandXboxController(
		OperatorConstants.driverControllerPort);
	private final CommandXboxController operatorController = new CommandXboxController(
		OperatorConstants.operatorControllerPort);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);

		swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem, driverController.getHID()));

		configureBindings();
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
		// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
		// new Trigger(exampleSubsystem::exampleCondition)
		// .onTrue(new ExampleCommand(exampleSubsystem));
		// Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed, cancelling on release
		// driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());

		/* driver */
		driverController.b().onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));

		/* operator */
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(exampleSubsystem);
	}
}
