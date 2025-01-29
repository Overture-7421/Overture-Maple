package overture.sim.mechanisms.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import overture.sim.NTMotor;
import overture.sim.mechanisms.SimMechanism;
import overture.sim.robots.SimBaseRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulates a flywheel mechanism
 */
public class Flywheel extends SimMechanism {
    private NTMotor motor;
    private FlywheelSim flywheelSim;
    private Rotation3d rotationAxis;
    private double angle;

    /**
     * Creates a new Flywheel
     * @param robot The robot that this flywheel is attached to
     * @param robotToWheel The transform from the robot to the flywheel
     * @param rotationAxis The axis of rotation of the flywheel
     * @param name The name of the flywheel
     * @param gearbox The gearbox of the flywheel
     * @param gearing  The gearing of the flywheel
     * @param jMomentofIntertia The moment of inertia of the flywheel
     * @param miAngle  The minimum angle of the flywheel
     * @param maxAngle The maximum angle of the flywheel
     * @param startingAngle The starting angle of the flywheel
     * @param inverted If the motor is inverted
     */
    public Flywheel(SimBaseRobot robot,
            Transform3d robotToWheel,
            Rotation3d rotationAxis,
            String name,
            DCMotor gearbox,
            double gearing,
            double jMomentofIntertia,
            boolean gravity,
            boolean inverted) {
        super(new Transform3d(robotToWheel.getTranslation(), robotToWheel.getRotation()));
        flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox, jMomentofIntertia, gearing), gearbox);
        this.rotationAxis = rotationAxis;


        motor = new NTMotor(new NTMotor.Config() {
            {
              Name = robot.GetName() + "/motors/" + name;
              VoltageApplied = (voltage) -> flywheelSim.setInputVoltage(voltage.in(Volts));
              EncoderPosition = () -> Radians.of(angle);
              EncoderSpeed = ()  -> RadiansPerSecond.of(flywheelSim.getAngularVelocityRadPerSec()).times(gearing);
              Current = () -> Amps.of(flywheelSim.getCurrentDrawAmps());
              Inverted = inverted;
            }
          });
    }

    public void Update() {

        double timeStep = GetTimeStep();
        flywheelSim.update(timeStep);
        motor.Update();
        double deltaAngle = flywheelSim.getAngularVelocityRadPerSec() * timeStep;

        angle += deltaAngle;
    }


    @Override
public List<Pose3d> GetPoses3d() {


    Rotation3d rotation = new Rotation3d(
        rotationAxis.getX() * angle,
        rotationAxis.getY() * angle,
        rotationAxis.getZ() * angle
    );

    return List.of(new Pose3d()
        .transformBy(GetRobotToMechanism())
        .transformBy(new Transform3d(Meters.of(0), Meters.of(0), Meters.of(0), rotation)));
}
}