package overture.sim.mechanisms.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import overture.sim.NTMotor;
import overture.sim.mechanisms.SimMechanism;
import overture.sim.robots.SimBaseRobot;

/**
 * Simulates an elevator mechanism
 */
public class Elevator extends SimMechanism {
  private NTMotor motor;
  private ElevatorSim elevatorSim;
  private Translation3d slideAxis;

  /**
   * Creates a new Elevator
   * @param robot The robot that this elevator is attached to
   * @param robotToElevator The transform from the robot to the elevator
   * @param slideAxis The axis the elevator moves on
   * @param name The name of the elevator
   * @param gearbox The gearbox of the elevator
   * @param gearing The gearing of the elevator
   * @param carriageMassKg The mass of the carriage
   * @param drumRadiusMeters The radius of the drum
   * @param minHeightMeters The minimum height of the elevator
   * @param maxHeightMeters The maximum height of the elevator
   * @param startingHeightMeters The starting height of the elevator
   * @param inverted If the motor is inverted
   */
  public Elevator(SimBaseRobot robot,
      Transform3d robotToElevator,
      Translation3d slideAxis,
      String name,
      DCMotor gearbox,
      double gearing,
      Mass carriageMassKg,
      Distance drumRadiusMeters,
      Distance minHeightMeters,
      Distance maxHeightMeters,
      Distance startingHeightMeters,
      boolean inverted) {
    super(new Transform3d(robotToElevator.getTranslation(), robotToElevator.getRotation()));
    elevatorSim = new ElevatorSim(gearbox, gearing, carriageMassKg.in(Kilograms), drumRadiusMeters.in(Meters),
        minHeightMeters.in(Meters), maxHeightMeters.in(Meters), true, startingHeightMeters.in(Meters));
    this.slideAxis = slideAxis;
    double metersToEncoderFactor = 1.0 / (drumRadiusMeters.in(Meters) * 2.0 * Math.PI) * gearing;

    motor = new NTMotor(new NTMotor.Config() {
      {
        Name = robot.GetName() + "/motors/" + name;
        VoltageApplied = (voltage) -> elevatorSim.setInputVoltage(voltage.in(Volts));
        EncoderPosition = () -> Revolutions.of(elevatorSim.getPositionMeters() * metersToEncoderFactor);
        EncoderSpeed = () -> RevolutionsPerSecond.of(elevatorSim.getVelocityMetersPerSecond() * metersToEncoderFactor);
        Current = () -> Amps.of(elevatorSim.getCurrentDrawAmps());
        Inverted = inverted;
      }
    });
  }

  @Override
  public void Update() {
    elevatorSim.update(GetTimeStep());
    motor.Update();
  }

  @Override
  public Pose3d GetPose3d() {
    return new Pose3d().transformBy(GetRobotToMechanism()).transformBy(
        new Transform3d(Meters.of(elevatorSim.getPositionMeters() * slideAxis.getX()), Meters.of(elevatorSim.getPositionMeters() * slideAxis.getY()), Meters.of(elevatorSim.getPositionMeters() * slideAxis.getZ()), new Rotation3d()));
  }
}