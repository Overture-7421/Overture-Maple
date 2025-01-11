package overture.sim;

import static edu.wpi.first.units.Units.Seconds;

import java.util.EnumSet;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import overture.sim.robots.Reefscape2025;
import overture.sim.robots.SimBaseRobot;

public class SimMain {
	static SimulatedArena arena = null;
	static SimBaseRobot robot = null;

	NetworkTableListener autoLoad = null;

	public void Initialize(LoggedRobot loggedRobot) {
		// Mechanisms
		robot = new Reefscape2025("Reefscape2025", new Pose2d(3, 3, new Rotation2d()));

		// Arena
		SimulatedArena.overrideInstance(new Arena2025Reefscape());
		SimulatedArena.overrideSimulationTimings(Seconds.of(loggedRobot.getPeriod()), 300);
		arena = SimulatedArena.getInstance();


		arena.resetFieldForAuto();

		// Add mechanisms to Arena
		arena.addDriveTrainSimulation(robot.GetDriveTrain());

		autoLoad = NetworkTableListener.createListener(
				NetworkTableInstance.getDefault().getStringTopic("/PathPlanner/activePath"),
				EnumSet.of(NetworkTableEvent.Kind.kValueRemote), SimMain::handleAutoChange);
	}

	static public void handleAutoChange(NetworkTableEvent event) {
		try {
			StructArraySubscriber<Pose2d> initPoseArray = event.getInstance().getTable("PathPlanner")
					.getStructArrayTopic("activePath", Pose2d.struct).subscribe(new Pose2d[1]);
			Pose2d initPose = initPoseArray.get()[0];

			robot.GetDriveTrain().setSimulationWorldPose(initPose);
			arena.resetFieldForAuto();
			DriverStation.reportWarning("Auto Loaded!", false);

		} catch (Exception e) {
			DriverStation.reportError("Can't update robot Pose2d", false);
		}
	}

	public void Periodic() {
		arena.simulationPeriodic();
		robot.Update();

		Logger.recordOutput("FieldSimulation/RobotPosition", robot.GetDriveTrain().getSimulatedDriveTrainPose());

		Logger.recordOutput("FieldSimulation/FinalComponentPoses", robot.GetMechanismPoses());
		Logger.recordOutput("FieldSimulation/ZeroedComponentPoses", robot.GetZeroedMechanismPoses());

		Logger.recordOutput("FieldSimulation/Algae",
				SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
		Logger.recordOutput("FieldSimulation/Coral",
				SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
	}
}
