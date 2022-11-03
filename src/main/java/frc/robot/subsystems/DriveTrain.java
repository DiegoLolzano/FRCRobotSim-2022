// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  //Declaraciones de motores
  WPI_TalonSRX leftFront = new WPI_TalonSRX(0);
  WPI_TalonSRX leftRear = new WPI_TalonSRX(1);
  WPI_TalonSRX rightFront = new WPI_TalonSRX(2);
  WPI_TalonSRX rightRear = new WPI_TalonSRX(3);

  //Estos objetos pueden recibir o regresar valores de entrada de motores
  TalonSRXSimCollection leftSim = leftFront.getSimCollection();
  TalonSRXSimCollection rightSim = rightFront.getSimCollection();

  //Permite que multiples objetos tipo MotorController (Controladores de motores) esten vinculados
  private final MotorControllerGroup m_leftMotors = 
  new MotorControllerGroup(leftFront, leftRear);

  private final MotorControllerGroup m_rightMotors = 
  new MotorControllerGroup(rightFront, rightRear);

  //Permite que el robot se mueva
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  
  //Encoders físicos
  private Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
  private Encoder m_rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);

  //Giroscopio físico
  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  //Clase para la odometria (posición en la cancha)
  private final DifferentialDriveOdometry m_odometry;

  //Esta clase simula el estado de un drivetrain
  public DifferentialDrivetrainSim m_drivetrainSim;

  //Encoders simulados
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  
  //Representación 2D de la cancha de juego para dashboards
  private Field2d m_fieldSim;

  //Giroscopio simulado
  private ADXRS450_GyroSim m_gyroSim;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    leftFront.configFactoryDefault();
    leftRear.configFactoryDefault();

    rightFront.configFactoryDefault();
    rightRear.configFactoryDefault();

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    if (RobotBase.isSimulation()) { 
      // Si nuestro robot es simulado
      // Esta clase simula el movimiento de nuestro drivetrain en la cancha.
      m_drivetrainSim =
          new DifferentialDrivetrainSim(
              DriveConstants.kDrivetrainPlant,
              DriveConstants.kDriveGearbox,
              DriveConstants.kDriveGearing,
              DriveConstants.kTrackwidthMeters,
              DriveConstants.kWheelDiameterMeters / 2.0,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
    }

    /****** METODOS CONSTRUCTORES ******/
    m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    m_rightEncoderSim = new EncoderSim(m_rightEncoder);

    m_gyroSim = new ADXRS450_GyroSim(m_gyro);

    m_fieldSim = new Field2d();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    m_fieldSim.setRobotPose(getPose());

    SmartDashboard.putData("Field", m_fieldSim);

    SmartDashboard.putNumber("Angulo del robot", getHeading());

    
  }

  @Override
  public void simulationPeriodic() {
    // Para actualizar nuestra simulación, establecemos las entradas de voltaje del robot, actualizar la simulación 
    // y escribir las posiciones y velocidades simuladas a nuestro encoder y giroscopio simulado.
    // Se pone negativo el lado derecho para que voltajes positivos hagan que el lado derecho
    // se mueva hacia adelante.
    
    m_drivetrainSim.setInputs(
        m_leftMotors.get() * RobotController.getBatteryVoltage(),
        -m_rightMotors.get() * RobotController.getBatteryVoltage());
    m_drivetrainSim.update(0.020);

    /*
    leftSim.setQuadratureRawPosition(distanceToNativeUnits(m_drivetrainSimulator.getLeftPositionMeters()));
    leftSim.setQuadratureVelocity(velocityToNativeUnits(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
    rightSim.setQuadratureRawPosition(distanceToNativeUnits(m_drivetrainSimulator.getRightPositionMeters()));
    rightSim.setQuadratureRawPosition(velocityToNativeUnits(m_drivetrainSimulator.getRightVelocityMetersPerSecond()));
    */
    m_leftEncoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSim.getRightVelocityMetersPerSecond());

    /*
    leftSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightSim.setBusVoltage(RobotController.getBatteryVoltage());
    */

    m_gyroSim.setAngle(-m_drivetrainSim.getHeading().getDegrees());
  }

  /**
   * Regresa la corriente consumida por el drivetrain. Esto solo funciona en ¡SIMULACIÓN! Si quieres que
   * funcione en cualquier otra cosa, use el codigo en: {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return La corriente consumida en Amperes
   */
  public double getDrawnCurrentAmps() {
    return m_drivetrainSim.getCurrentDrawAmps();
  }

  /**
   * Regresa la posición estimada actual del robot.
   *
   * @return La posición
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    //resetEncoders();
    m_drivetrainSim.setPose(pose);
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void setTeleopControl(double speed, double turn){

    double left = speed - turn;
    double right = speed + turn;

    leftFront.set(left);
    rightFront.set(right);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360)
        * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /****** CONVERSIONES ******/
  //Distancia a unidades de encoder
  private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * DriveConstants.kWheelDiameterMeters);
    double motorRotations = wheelRotations * DriveConstants.kDriveGearing;
    int sensorCounts = (int)(motorRotations * DriveConstants.kEncoderCPR);
    return sensorCounts;
  }

  //Unidades de encoder a Distancia en metros
    private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / DriveConstants.kEncoderCPR;
    double wheelRotations = motorRotations / DriveConstants.kDriveGearing;
    double positionMeters = wheelRotations * (2 * Math.PI * DriveConstants.kWheelDiameterMeters);
    return positionMeters;
  }

  //Velocidad (metros por segundo) a unidades de encoder
  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * DriveConstants.kWheelDiameterMeters);
    double motorRotationsPerSecond = wheelRotationsPerSecond * DriveConstants.kDriveGearing;
    double motorRotationsPer100ms = motorRotationsPerSecond / DriveConstants.k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * DriveConstants.kEncoderCPR);
    return sensorCountsPer100ms;
  }
}

