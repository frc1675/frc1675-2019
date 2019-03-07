/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utils;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class DPadButton extends Button {

    Joystick joystick;
    Direction direction;

    public DPadButton(Joystick joystick, Direction direction) {
        this.joystick = joystick;
        this.direction = direction;
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }

    public boolean get() {
        int DPadValue = joystick.getPOV();
        return (DPadValue == direction.direction) || (DPadValue == (direction.direction + 45) % 360)
                || (DPadValue == (direction.direction + 315) % 360);
    }



}
/**
 * Add your docs here.
 */

