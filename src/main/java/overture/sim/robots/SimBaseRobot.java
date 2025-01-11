package overture.sim.robots;

import java.util.List;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import overture.sim.mechanisms.SimMechanism;

public abstract class SimBaseRobot {
    private String name;

    public SimBaseRobot(String name, Pose2d startingPose) {
        this.name = name;
    }

    public String GetName() {
        return name;
    }

    public Pose3d[] GetMechanismPoses(){
        return GetMechanisms().stream().map(SimMechanism::GetPose3d).toArray(Pose3d[]::new);
    }

    public Pose3d[] GetZeroedMechanismPoses(){
        return GetMechanisms().stream().map(mechanism -> new Pose3d()).toArray(Pose3d[]::new);
    }

    public abstract void Update();
    public abstract AbstractDriveTrainSimulation GetDriveTrain();
    public abstract List<SimMechanism> GetMechanisms();
}
