package overture.sim.robots;

import java.util.ArrayList;
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
        ArrayList<Pose3d> poses = new ArrayList<>();
        
        for (SimMechanism mechanism : GetMechanisms()) {
            for (Pose3d pose : mechanism.GetPoses3d()) {
                poses.add(pose);
            }
        }

        return poses.toArray(Pose3d[]::new);
    }

    public Pose3d[] GetZeroedMechanismPoses(){
        ArrayList<Pose3d> poses = new ArrayList<>();
        
        for (SimMechanism mechanism : GetMechanisms()) {
            for (int i = 0; i < mechanism.GetPoses3d().size(); i++) {
                poses.add(new Pose3d());
            }
        }

        return poses.toArray(Pose3d[]::new);
    }

    public abstract void Update();
    public abstract AbstractDriveTrainSimulation GetDriveTrain();
    public abstract List<SimMechanism> GetMechanisms();
}
