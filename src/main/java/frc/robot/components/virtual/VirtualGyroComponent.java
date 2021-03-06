/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.virtual;

import frc.robot.components.GyroComponent;

/**
 * A virtual {@link GyroComponent} component.
 */
public class VirtualGyroComponent implements GyroComponent {

    private double angle;
    private double offset;

    @Override
    public double getAngle() {
        return angle + offset;
    }

    @Override
    public void reset() {
        offset = angle;
        angle = 0;
    }
}
