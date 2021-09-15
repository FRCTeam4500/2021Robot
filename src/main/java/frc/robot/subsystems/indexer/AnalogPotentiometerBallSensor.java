// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

/** Add your docs here. */
public class AnalogPotentiometerBallSensor extends AnalogPotentiometer implements BallSensor{

    double threshold = 1;

    public AnalogPotentiometerBallSensor(AnalogInput input) {
        super(input);
    }

    @Override
    public boolean registersBall() {
        return get() <= threshold;
    }

    public AnalogPotentiometerBallSensor(int channel) {
        super(channel);
    }

    public AnalogPotentiometerBallSensor(int channel, double scale) {
        super(channel, scale);
    }

    public AnalogPotentiometerBallSensor(AnalogInput input, double scale) {
        super(input, scale);
    }

    public AnalogPotentiometerBallSensor(int channel, double fullRange, double offset) {
        super(channel, fullRange, offset);
    }

    public AnalogPotentiometerBallSensor(AnalogInput input, double fullRange, double offset) {
        super(input, fullRange, offset);
    }

    public AnalogPotentiometerBallSensor(int channel, double fullRange, double offset, double threshold) {
        super(channel, fullRange, offset);
        this.threshold = threshold;
    }

    public AnalogPotentiometerBallSensor(AnalogInput input, double fullRange, double offset, double threshold) {
        super(input, fullRange, offset);
        this.threshold = threshold;
    }

    public double getThreshold() {
        return threshold;
    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }
    @Override
    public double get() {
        // TODO Auto-generated method stub
        return -super.get();
    }
}