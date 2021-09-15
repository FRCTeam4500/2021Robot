package frc.robot.subsystems.indexer.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.indexer.AnalogPotentiometerBallSensor;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.NetworkTableBallSensor;

public class HardwareIndexerFactory {
    /**
     *
     */
    private static final int BALL_SENSOR_THRESHOLD = 40;
    private static final int CHANNEL = 1;
    private static final double FULL_RANGE = 36, OFFSET = -36, MAIN_THRESHOLD = 24;
    /**
     *
     */
    private static final int INDEXER_MOTOR_PORT = 12;
    private static AnalogPotentiometerBallSensor lastCreatedSensor;
    public static Indexer makeIndexer(){
        lastCreatedSensor = new AnalogPotentiometerBallSensor(CHANNEL, FULL_RANGE, OFFSET, MAIN_THRESHOLD);
        return new Indexer(
                new TalonSRXComponent(INDEXER_MOTOR_PORT),
                lastCreatedSensor,
                new NetworkTableBallSensor("Sensor1", BALL_SENSOR_THRESHOLD+15),
                new NetworkTableBallSensor("Sensor2", BALL_SENSOR_THRESHOLD),
                new NetworkTableBallSensor("Sensor3", BALL_SENSOR_THRESHOLD),
                new NetworkTableBallSensor("Sensor4", BALL_SENSOR_THRESHOLD),
                new NetworkTableBallSensor("Sensor5", BALL_SENSOR_THRESHOLD)
        );
    }
    public static AnalogPotentiometerBallSensor getAnalogSensor(){
        return lastCreatedSensor;
    }
}