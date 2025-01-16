package overture.sim.mechanisms;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public abstract class SimMechanism {
    private Transform3d robotToMechanism;
    private double lastTime;

    public SimMechanism(Transform3d robotToMechanism) {
        this.robotToMechanism = robotToMechanism;
        lastTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    protected final double GetTimeStep() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double timeStep = currentTime - lastTime;
        lastTime = currentTime;
        return timeStep;
    }
    public final void SetRobotToMechanism(Transform3d robotToMechanism) {
        this.robotToMechanism = robotToMechanism;
    }

    public final Transform3d GetRobotToMechanism() {
        return robotToMechanism;
    }

    public abstract void Update();

    public abstract List<Pose3d> GetPoses3d();
}
