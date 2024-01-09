package swervelib.parser;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import swervelib.parser.deserializer.PIDFRange;

/**
 * Hold the PIDF and Integral Zone values for a PID.
 */
public class ProfiledPIDFConfig
{

  /**
   * Proportional Gain for PID.
   */
  public double p;
  /**
   * Integral Gain for PID.
   */
  public double i;
  /**
   * Derivative Gain for PID.
   */
  public double d;
  /**
   * Feedforward value for PID.
   */
  public double f;
  /**
   * Integral zone of the PID.
   */
  public double iz;

  /**
   * Max speed for trapezoidal motoion profiling
   */
  public double maxSpeed;

  /**
   * Max acceleration for trapezoidal motion profiling
   */

  public double maxAccel;

  /**
   * The PIDF output range.
   */
  public PIDFRange output = new PIDFRange();

  /**
   * Used when parsing PIDF values from JSON.
   */
  public ProfiledPIDFConfig()
  {
  }

  /**
   * PIDF Config constructor to contain the values.
   *
   * @param p  P gain.
   * @param i  I gain.
   * @param d  D gain.
   * @param f  F gain.
   * @param iz Intergral zone.
   * @param maxSpeed max speed for trapezoidal motion profiling.
   * @param maxAccel max acceleration for trapezoidal motion profiling.
   */
  public ProfiledPIDFConfig(double p, double i, double d, double f, double iz, double maxSpeed, double maxAccel)
  {
    this.p = p;
    this.i = i;
    this.d = d;
    this.f = f;
    this.iz = iz;
    this.maxSpeed = maxSpeed;
    this.maxAccel = maxAccel;
  }

  /**
   * Profiled PIDF Config constructor to contain the values.
   *
   * @param p P gain.
   * @param i I gain.
   * @param d D gain.
   * @param f F gain.
   * @param maxSpeed max speed for trapezoidal motion profiling.
   * @param maxAccel max acceleration for trapezoidal motion profiling.
   */
  public ProfiledPIDFConfig(double p, double i, double d, double f, double maxSpeed, double maxAccel)
  {
    this(p, i, d, f, 0, maxSpeed, maxAccel);
  }

  /**
   * Profiled PIDF Config constructor to contain the values.
   *
   * @param p P gain.
   * @param i I gain.
   * @param d D gain.
   * @param maxSpeed max speed for trapezoidal motion profiling.
   * @param maxAccel max acceleration for trapezoidal motion profiling.
   */
  public ProfiledPIDFConfig(double p, double i, double d, double maxSpeed, double maxAccel)
  {
    this(p, i, d, 0, 0, maxSpeed, maxAccel);
  }

  /**
   * Profiled PIDF Config constructor to contain the values.
   *
   * @param p P gain.
   * @param d D gain.
   * @param maxSpeed max speed for trapezoidal motion profiling.
   * @param maxAccel max acceleration for trapezoidal motion profiling.
   */
  public ProfiledPIDFConfig(double p, double d, double maxSpeed, double maxAccel)
  {
    this(p, 0, d, 0, 0, maxSpeed, maxAccel);
  }

  /**
   * Create a {@link ProfiledPIDController} from the PID values.
   *
   * @return PIDController.
   */
  public ProfiledPIDController createPIDController()
  {
    return new ProfiledPIDController(p, i, d, new Constraints(maxSpeed, maxAccel));
  }
}