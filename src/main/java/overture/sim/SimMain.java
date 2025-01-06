package overture.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import overture.sim.swerve.SwerveChassis;

public class SimMain {
    SimulatedArena arena = null;
    SwerveChassis chassis = null;
    VisionSystemSim visionSim = new VisionSystemSim("main");

    PhotonCamera camera = new PhotonCamera("testCamera");

    public void Initialize() {
        // Mechanisms
        chassis = new SwerveChassis("Offseason 2024", new Pose2d(3, 3, new Rotation2d()));

        // Arena
        SimulatedArena.overrideInstance(new Arena2024Crescendo());
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        visionSim.addAprilTags(tagLayout);
        AddSimCameras();
        arena = SimulatedArena.getInstance();

        // Add game pieces to Arena
        arena.addGamePiece(new CrescendoNoteOnField(new Translation2d(3, 3)));

        // Add mechanisms to Arena
        arena.addDriveTrainSimulation(chassis);
    }

    public void AddSimCameras() {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // The PhotonCamera used in the real robot code.
        // The simulation of this camera. Its values used in real robot code will be updated.
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);

        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        // Add this camera to the vision system simulation with the given robot-to-camera transform.
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    public void Periodic() {
        arena.simulationPeriodic();
        chassis.Update();
        visionSim.update(chassis.getSimulatedDriveTrainPose());

        Logger.recordOutput("FieldSimulation/RobotPosition", chassis.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Notes", arena.getGamePiecesArrayByType("Note"));
    }
}
