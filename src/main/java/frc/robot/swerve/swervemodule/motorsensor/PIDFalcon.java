package frc.robot.swerve.swervemodule.motorsensor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.swerve.util.constants.PIDPreferenceConstants;

/**
 * PIDMotor implementation for the NEO. Uses the built-in encoder and PID on the
 * Spark Max.
 */
public class PIDFalcon extends TalonFX implements PIDMotor {

    // The offset to add to position values, in rotations.
    private double offset = 0;

    /**
     * Constructor. Sets up the default configuration for a SparkMax.
     * 
     * @param canID
     *                         The canID for the SparkMax
     * @param pidConstants
     *                         All of the PID constants for velocity control. Uses
     *                         all constants except tolerance
     */
    public PIDFalcon(int canID, PIDPreferenceConstants pidConstants) {
        super(canID);

        this.configFactoryDefault();
        this.setNeutralMode(NeutralMode.Brake);
        this.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        pidConstants.getKP().addChangeHandler(this::setKP);
        pidConstants.getKI().addChangeHandler(this::setKI);
        pidConstants.getKD().addChangeHandler(this::setKD);
        pidConstants.getKF().addChangeHandler(this::setKF);
        pidConstants.getIZone().addChangeHandler(this::setIZone);
        pidConstants.getIMax().addChangeHandler(this::setIMax);
        this.config_kP(0, pidConstants.getKP().getValue());
        this.config_kI(0, pidConstants.getKI().getValue());
        this.config_kD(0, pidConstants.getKD().getValue());
        this.config_kF(0, pidConstants.getKF().getValue());
        this.config_IntegralZone(0, pidConstants.getIZone().getValue());
        this.setIMax(pidConstants.getIMax().getValue());
    }

    /**
     * {@inheritDoc}
     * 
     * @return The motor shaft position in rotations
     */
    @Override
    public double getPosition() {
        return this.getSelectedSensorPosition() + offset;
    }

    /**
     * {@inheritDoc}
     * 
     * @return The motor shaft position in rotations per second
     */
    @Override
    public double getVelocity() {
        return this.getSelectedSensorVelocity() / 60.;
    }

    /**
     * {@inheritDoc}
     * 
     * @return The position to set, in rotations
     */
    @Override
    public void calibratePosition(double position) {
        this.offset = position - this.getPosition() + this.offset;
    }

    /**
     * {@inheritDoc}
     * 
     * @return The velocity to set, in rotations per second
     */
    @Override
    public void setVelocity(double velocity) {
        this.set(ControlMode.Velocity, velocity * 60.);
    }

    /**
     * Set the kP constant for the Spark Max velocity control.
     * 
     * @param kP
     *               The proportional gain
     */
    private void setKP(double kP) {
        this.config_kP(0, kP);
    }

    /**
     * Set the kI constant for the Spark Max velocity control.
     * 
     * @param kI
     *               The integral gain
     */
    private void setKI(double kI) {
        this.config_kI(0, kI);
    }

    /**
     * Set the kD constant for the Spark Max velocity control.
     * 
     * @param kD
     *               The differential gain
     */
    private void setKD(double kD) {
        this.config_kD(0, kD);
    }

    /**
     * Set the kF constant for the Spark Max velocity control.
     * 
     * @param kF
     *               The feedforward gain
     */
    private void setKF(double kF) {
        this.config_kF(0, kF);
    }

    /**
     * Set the iZone constant for the Spark Max velocity control.
     * 
     * @param iZone
     *                  The max error which will accumulate in the integral
     */
    private void setIZone(double iZone) {
        this.config_IntegralZone(0, iZone);
    }

    /**
     * Set the iMax constant for the Spark Max velocity control.
     * 
     * @param iMax
     *                 The max accumulated error for the integral
     */
    private void setIMax(double iMax) {
        this.setIMax(iMax);
    }

}