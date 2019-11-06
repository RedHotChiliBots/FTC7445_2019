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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Arrays;
import java.util.List;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware {

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQvZgMD/////AAABmZquHHM/akuGkmTcIcssi+gINVzua6tbuI6iq9wY3ypvUkndXoRQncprZtLgjoNzaAZx4jTucekE90oZj0G/CqgXL1uzhrV4+knSziKUwgFVy3SVvGzw0+/ZqHVFwAFe6wsty2B2Mxg+uIoAFq7tB5WRB6GMx1j47m9q7+hkx3+KOKasSiO/T8Fd/nehQkRVBwB1XJNEo28R0yicJfdGkhxgJOK/CGTkN49MooMjaSx1PFpgx2Bx8wxJwNMcOxzh3zYeiwddMZvsycSf3h2WTDHBHeFkW+f00i0071LJRaawELtRmIxP/pmV2Squ/1daGYjLGKveSPH5tBIHiQvGwdAnv3QrRZnhEf6ztG9eELEs";

    public VuforiaLocalizer.Parameters parameters;
//    private WebcamName webcamName = null;


    /* Public OpMode members. */
    public DcMotor leftFrontDrive   = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor leftRearDrive    = null;
    public DcMotor rightRearDrive   = null;

    public Servo leftFoundationServo  = null;
    public Servo rightFoundationServo = null;

    public Servo parkArmServo = null;

    public Servo capGuardServo = null;
    public Servo capReleaseServo = null;

    public DcMotor leftStoneGrabber  = null;
    public DcMotor rightStoneGrabber = null;

    public final double LEFT_UP  = 0.75;
    public final double LEFT_DN  = 0.25;
    public final double RIGHT_UP  = 0.0;
    public final double RIGHT_DN  = 0.5;

    public final double PARK_UP  = 0.75;
    public final double PARK_DN  = 0.0;

    public final double GUARD_CLOSE  = 0.25;
    public final double GUARD_OPEN  = 0.75;
    public final double CAP_STOW  = 0.0;
    public final double CAP_RELEASE  = 0.75;

    public final double LEFT_IN   = 1.0;
    public final double LEFT_OUT  = -1.0;
    public final double RIGHT_IN  = -1.0;
    public final double RIGHT_OUT = 1.0;

    public enum FDTN {UP, DOWN, OTHER};
    public enum PARK {UP, DOWN, OTHER};
    public enum CAP  {STOW, RELEASE, OTHER};
    public enum COLOR {RED,BLUE,OTHER}
    public enum TRACK {TRACKING,STOPPED,UNKNOWN}

    private FDTN fntnPosition = FDTN.UP;
    private PARK parkPosition = PARK.UP;
    private CAP  capPosition = CAP.STOW;

    private TRACK trackState = TRACK.UNKNOWN;

    private boolean stoneDir = true;
    private boolean driveDir = true;
    private boolean driveHalfSpeed = false;

    private double leftDrive = 0.0;
    private double rightDrive = 0.0;

    private double leftStone = 0.0;
    private double rightStone = 0.0;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /***************************************************************/
        /*********** Define and Initialize Vuforia & Camera ************/
        /***************************************************************/

//        webcamName = ahwMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
//        parameters.cameraName = webcamName;

        /***********************************************************/
        /*********** Define and Initialize Drive Motors ************/
        /***********************************************************/
        // Define each drive motor
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hwMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hwMap.get(DcMotor.class, "rightRearDrive");

        // Initialize drive motors to correct rotation
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Initialize all drive motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        // Initialize all drive motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /***********************************************************/
        /************** Define and Initialize Servos ***************/
        /***********************************************************/

        leftFoundationServo = hwMap.get(Servo.class, "leftFoundationServo");
        rightFoundationServo = hwMap.get(Servo.class, "rightFoundationServo");

        setFoundation(FDTN.UP);

        parkArmServo = hwMap.get(Servo.class, "parkArmServo");

        setParkArm(PARK.UP);

        capGuardServo = hwMap.get(Servo.class, "capGuardServo");
        capReleaseServo = hwMap.get(Servo.class, "capReleaseServo");

        setCapGuard(CAP.STOW);
        setCapRelease(CAP.STOW);

        leftStoneGrabber = hwMap.get(DcMotor.class, "leftStoneGrabber");
        rightStoneGrabber = hwMap.get(DcMotor.class, "rightStoneGrabber");

        leftStoneGrabber.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightStoneGrabber.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftStoneGrabber.setPower(0);
        rightStoneGrabber.setPower(0);

        leftStoneGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightStoneGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setFoundation(Hardware.FDTN pos) {
        fntnPosition = pos;
        if (pos == FDTN.UP) {
            leftFoundationServo.setPosition(LEFT_UP);
            rightFoundationServo.setPosition(RIGHT_UP);
        } else {
            leftFoundationServo.setPosition(LEFT_DN);
            rightFoundationServo.setPosition(RIGHT_DN);
        }
    }

    public Hardware.FDTN getFoundation() {
        return fntnPosition;
    }

    public void setParkArm(Hardware.PARK pos) {
        parkPosition = pos;
        if (pos == PARK.UP) {
            parkArmServo.setPosition(PARK_UP);
        } else {
            parkArmServo.setPosition(PARK_DN);
        }
    }

    public Hardware.PARK getParkArm() {
        return parkPosition;
    }

     public void setCapGuard(CAP c) {
        capPosition = c;
        if (c == CAP.STOW) {
            capGuardServo.setPosition(GUARD_CLOSE);
        } else {
            capGuardServo.setPosition(GUARD_OPEN);
        }
     }

    public void setCapRelease(CAP c) {
        capPosition = c;
        if (c == CAP.STOW) {
            capReleaseServo.setPosition(CAP_STOW);
        } else {
            capReleaseServo.setPosition(CAP_RELEASE);
        }
    }

    public void setDriveSpeed(double l, double r) {
        if (driveHalfSpeed) {
            l /= 2.0;
            r /= 2.0;
        }
        leftDrive = l;
        rightDrive = r;
        if (driveDir) {
            leftFrontDrive.setPower(l);
            rightFrontDrive.setPower(r);
            leftRearDrive.setPower(l);
            rightRearDrive.setPower(r);
        } else {
            leftFrontDrive.setPower(-r);
            rightFrontDrive.setPower(-l);
            leftRearDrive.setPower(-r);
            rightRearDrive.setPower(-l);
        }
    }

    public List<Double> getDriveSpeed() {
        return Arrays.asList(leftDrive, rightDrive);
    }

    public void setDriveHalfSpeed(boolean h) {
        driveHalfSpeed = h;
    }

    public boolean getDriveHalfSpeed() {
        return driveHalfSpeed;
    }

    public void setStoneSpeed(double l, double r) {
        if (stoneDir) {
            leftStone = l;
            rightStone = r;
        } else {
            leftStone = -l;
            rightStone = -r;
        }
        leftStoneGrabber.setPower(leftStone);
        rightStoneGrabber.setPower(rightStone);
    }

    public List<Double> getStoneSpeed() {
        return Arrays.asList(leftStone, rightStone);
    }

    public void setDriveDir(boolean d) {
        driveDir = !driveDir;
    }

    public boolean getDriveDir() {
        return driveDir;
    }

    public void setStoneDir(boolean d) {
        stoneDir = !stoneDir;
    }

    public boolean getStoneDir() {
        return stoneDir;
    }

    public void setTrackState(TRACK s) {
        trackState = s;
    }

    public TRACK getTrackState() {
        return trackState;
    }
 }

