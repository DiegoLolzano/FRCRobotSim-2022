// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * La clase Constants provee un lugar conveniente para que los equipos mantengan valores numericos o booleanos constantes de todo el robot
 * Esta clase no debe ser usada para otro propsito. Todas las constantes deben ser declaradas
 * globalmente (Ej. public static). No ponga nada funcional en esta clase.
 *
 * <p>Se recomienda importar estaticamente esta clase(o alguna clase de su interior) donde sea que
 * las constantes sean necesitadas, para reducir verbosidad.
 */
public final class Constants {
    public static final class DriveConstants{

      //Puertos de encoder. Necesarios para el metodo constructor de este objeto
    public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};

    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int k100msPerSecond = 10;

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kEncoderDistancePerPulse = //0.001948;

        // Asume que los encoders estan montados direectamente en los ejes de las ruedas
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR * 12.0;

    public static final boolean kGyroReversed = true;

    // Estos son solo valores de ejemplos - ¡NO USE NINGUNO DE ESTOS PARA SU PROPIO ROBOT!
    // Estos valores de caracterización DEBEN de ser determinados ya sea de forma experimental o teorica
    // para *su* unidad de robot
    // El Conjunto de herramientas de caracterización de robots proporciona una herramienta conveniente para obtener estos
    // valores para su robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Estos son solo valores de ejemplos - ¡NO USE NINGUNO DE ESTOS PARA SU PROPIO ROBOT!
    // Estos valores de caracterización DEBEN de ser determinados ya sea de forma experimental o teorica
    // para *su* unidad de robot
    // Estos dos valores son kV and kA "angular"
    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
        LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian,
            kaVoltSecondsSquaredPerRadian);

    // Valores solamente de ejemplo -- ¡usa lo que esta en tu robot físico!
    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);
    public static final double kDriveGearing = 12;

    // Valores solamente de ejemplo - ¡esto debe ser afinado para su chasis!
    public static final double kPDriveVel = 0.1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.7;
    public static final double kMaxAccelerationMetersPerSecondSquared = 9.906;

    // Valores de referencia razonables para un follower RAMSETE en unidades de metros y segundos
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}