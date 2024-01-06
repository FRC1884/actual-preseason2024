package frc.robot.core.swerve.MAXSwerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.core.swerve.MAXSwerve.MaxSwerveConstants.*;

public abstract class MAXSwerve extends SubsystemBase {

  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(MaxSwerveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter =
      new SlewRateLimiter(MaxSwerveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private MAXSwerveModule fl, fr, bl, br;
  private ADIS16470_IMU gyro;

  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          MaxSwerveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(gyro.getAngle()),
          new SwerveModulePosition[] {
            fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()
          });

  public MAXSwerve(
      ADIS16470_IMU imu,
      MAXSwerveModule fl,
      MAXSwerveModule fr,
      MAXSwerveModule bl,
      MAXSwerveModule br) {
    this.gyro = imu;
    this.fl = fl;
    this.fr = fr;
    this.bl = bl;
    this.br = br;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
          fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()
        });
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
          fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate =
            Math.abs(MaxSwerveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif =
          MAXSwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir =
            MAXSwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = MAXSwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir =
            MAXSwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * MaxSwerveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        MaxSwerveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, MaxSwerveConstants.kMaxSpeedMetersPerSecond);
    fl.setDesiredState(swerveModuleStates[0]);
    fr.setDesiredState(swerveModuleStates[1]);
    bl.setDesiredState(swerveModuleStates[2]);
    br.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    fl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    fr.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    bl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    br.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, MaxSwerveConstants.kMaxSpeedMetersPerSecond);
    fl.setDesiredState(desiredStates[0]);
    fr.setDesiredState(desiredStates[1]);
    bl.setDesiredState(desiredStates[2]);
    br.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    fl.resetEncoders();
    fr.resetEncoders();
    bl.resetEncoders();
    br.resetEncoders();
  }

  /** Zeros the heading of the robot */
  public void ZeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (MaxSwerveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
