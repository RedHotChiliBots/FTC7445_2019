/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Library.Library;

import java.util.Arrays;
import java.util.List;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop Tank Linear", group="Teleop")
//@Disabled
public class TeleopTank_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot           = new Hardware(); // use the class created to define a Pushbot's hardware
    VuforiaSkyStoneWebcam vu = new VuforiaSkyStoneWebcam();
    Library lib = new Library();

    public void runOpMode() {
        initOpmode();
        waitForStart();
        startOpmode();
        while (opModeIsActive())
        {
            loopOpmode();
            telemetry.update();
        }
        stopOpMode();
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void initOpmode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        telemetry.addData("Hardware", "Init");

        vu.init(robot.parameters);
        vu.targetsSkyStone.activate();
        telemetry.addData("Vuforia", "Init");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");

        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
//    @Override
//    public void init_loop() {
//     }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void startOpmode() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loopOpmode() {
        List<Double> speed = Arrays.asList(0.0, 0.0);

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
//        left = -gamepad1.left_stick_y;
//        right = -gamepad1.right_stick_y;

        trackTarget(vu.stoneTarget);

        if (vu.getPosLOS() > 12.0) {
            // calculate course adjustment if
            // we are off coarse by 2 inches or 2 degrees
            speed = lib.calcCorrection(vu.getPosOffset(), vu.getPosAngle(), vu.getYaw());
        }

//        robot.leftFrontDrive.setPower((double)speed.get(0));
//        robot.rightFrontDrive.setPower((double)speed.get(1));
//        robot.leftRearDrive.setPower((double)speed.get(0));
//        robot.rightRearDrive.setPower((double)speed.get(1));

        // Use gamepad left & right Bumpers to open and close the claw
//        if (gamepad1.right_bumper)
//            clawOffset += CLAW_SPEED;
//        else if (gamepad1.left_bumper)
//            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
//        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//      robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
//        robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
//        if (gamepad1.y)
//            robot.leftArm.setPower(robot.ARM_UP_POWER);
//        else if (gamepad1.a)
//            robot.leftArm.setPower(robot.ARM_DOWN_POWER);
//        else
//            robot.leftArm.setPower(0.0);

        // Send telemetry message to signify robot running;
//        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("Speed",  "{left, right} = %4.2f %4.2f", (double)speed.get(0), (double)speed.get(1));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stopOpMode() {
        // Disable Tracking when we are done;
        vu.targetsSkyStone.deactivate();
    }

    private void trackTarget(VuforiaTrackable target) {

        if (((VuforiaTrackableDefaultListener)target.getListener()).isVisible()) {
            vu.setVisible(true);

            // getUpdatedRobotLocation() will return null if no new information is available since
            // the last time that call was made, or if the trackable is not currently visible.
            vu.setTransform(((VuforiaTrackableDefaultListener)target.getListener()).getUpdatedRobotLocation());

            telemetry.addData("Visible Target", target.getName());
            telemetry.addData("Pos (in)", "{Dist, Offset, Height} = %.1f, %.1f, %.1f", vu.getPosDist(), vu.getPosOffset(), vu.getPosHeight());
            telemetry.addData("Pos (in,deg)", "{LOS, Angle} = %.1f, %.1f", vu.getPosLOS(), vu.getPosAngle());
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Yaw} = %.0f, %.0f, %.0f", vu.getRoll(), vu.getPitch(), vu.getYaw());

        } else {
            vu.setVisible(false);
            vu.setTransform(null);
            telemetry.addData("Visible Target", "none");
        }
    }
}
