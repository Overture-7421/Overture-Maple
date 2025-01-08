package overture.sim;

import java.util.EnumSet;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import overture.sim.swerve.SwerveChassis;

public class SimMain {
	static SimulatedArena arena = null;
	static SwerveChassis chassis = null;

	NetworkTableListener autoLoad = null;

	public void Initialize() {
		// Mechanisms
		chassis = new SwerveChassis("Offseason 2024", new Pose2d(3, 3, new Rotation2d()));

		// Arena
		SimulatedArena.overrideInstance(new Arena2025Reefscape());
		arena = SimulatedArena.getInstance();

		// Add game pieces to Arena
		// arena.addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(3, 3)));

		arena.resetFieldForAuto();

		// Add mechanisms to Arena
		arena.addDriveTrainSimulation(chassis);

		autoLoad = NetworkTableListener.createListener(
				NetworkTableInstance.getDefault().getStringTopic("/PathPlanner/activePath"),
				EnumSet.of(NetworkTableEvent.Kind.kValueRemote), SimMain::handleAutoChange);
	}

	static public void handleAutoChange(NetworkTableEvent event) {
		try {
			StructArraySubscriber<Pose2d> initPoseArray = event.getInstance().getTable("PathPlanner")
					.getStructArrayTopic("activePath", Pose2d.struct).subscribe(new Pose2d[1]);
			Pose2d initPose = initPoseArray.get()[0];

			chassis.setSimulationWorldPose(initPose);
			arena.resetFieldForAuto();
			DriverStation.reportWarning("Auto Loaded!", false);

		} catch (Exception e) {
			DriverStation.reportError("Can't update robot Pose2d", false);
		}
	}

	public void Periodic() {
		arena.simulationPeriodic();
		chassis.Update();

		Logger.recordOutput("FieldSimulation/RobotPosition", chassis.getSimulatedDriveTrainPose());
		Logger.recordOutput("FieldSimulation/Algae",
				SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
		Logger.recordOutput("FieldSimulation/Coral",
				SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
		Logger.recordOutput("FieldSimulation/StackedAlgae",
				ReefscapeCoralAlgaeStack.getStackedAlgaePoses());
		Logger.recordOutput("FieldSimulation/StackedCoral",
				ReefscapeCoralAlgaeStack.getStackedCoralPoses());
	}
}
