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

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Library.MyOpMode;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="Linear Opmode")
public class Teleop extends MyOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;

    DcMotor liftLeft;
    DcMotor liftRight;

    DcMotor manip;
//
//    DcMotor relic;
//
//    Servo relicGrabber;
    CRServo jewelHand;
    CRServo jewelArm;

    float gamepad1_left;
    float gamepad1_right;

    double jewelHandStart;
//    double relicDeploy;
//    double relicGrab;

    int lessenPow = 0;

    double left = .5;
    double right = .5;
    double less = .25;
    double increase = .25;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

//        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
//        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
//        jewelColor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "jewelColor");
//
//        floorL = hardwareMap.get(OpticalDistanceSensor.class, "floorL");
//        floorR = hardwareMap.get(OpticalDistanceSensor.class, "floorR");
//
        liftLeft = hardwareMap.dcMotor.get("liftL");
        liftRight = hardwareMap.dcMotor.get("liftR");
//
//        relic = hardwareMap.dcMotor.get("relic");
//
        manip = hardwareMap.dcMotor.get("manip");
//
        jewelArm = hardwareMap.crservo.get("jewelArm");
        jewelHand = hardwareMap.crservo.get("jewelHand");

//        relicGrabber = hardwareMap.servo.get("relicGrabber");
//
//        telemetry.addData("Status", "Hardware Mapped");
//        telemetry.update();

        waitForStart();
        runtime.reset();

        //Change later
//        jewelHandStart = 0.0;
//        relicDeploy = 0.0;
//        relicGrab = 0.0;


        while (opModeIsActive()) {

            //setMotorsMecDPAD(.25, .25, .25, .25);

            //gamepad1 start
            gamepad1_left = gamepad1.left_stick_y;
            gamepad1_right = gamepad1.right_stick_y;

            // inverting front and back
            if (gamepad1.x) {
                gamepad1_left *= -1;
                gamepad1_right *= -1;
            }

            //reseting all values
            if (gamepad1.y) {
                gamepad1_left = gamepad1.left_stick_y;
                gamepad1_right = gamepad1.right_stick_y;
                //setMotorsMecDPAD(.25, .25, .25, .25);
                lessenPow = 0;
            }


            // Code for changing the driving speed by 0.5

            if (gamepad1.a && lessenPow == 0) {
                lessenPow = 1;
            }
            if (gamepad1.a && lessenPow == 1) {
                lessenPow = 0;
            }

            if (lessenPow == 1) {
                gamepad1_left *= .5;
                gamepad1_right *= .5;
            }
            if (lessenPow == 0) {
                gamepad1_left = gamepad1.left_stick_y;
                gamepad1_right = gamepad1.right_stick_y;
            }


            //Normal driving code with slowdown method
            if ((Math.abs(gamepad1.left_stick_y) > .05 || Math.abs(gamepad1.right_stick_y) > .05)) {
                motorBL.setPower(gamepad1_left);
                motorBR.setPower(-gamepad1_right);
                motorFL.setPower(gamepad1_left);
                motorFR.setPower(-gamepad1_right);
            } else {
                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
            }


            if (gamepad1.dpad_up &&  left <= .75 && right <= .75) {
                left += increase;
                right += increase;
                telemetry.addData("Left:", left);
                telemetry.addData("Right:", right);
                telemetry.update();
            }

            else if (gamepad1.dpad_down && left >= 0.25 && right >= 0.25){
                left += less;
                right += less;
                telemetry.addData("Left:", left);
                telemetry.addData("Right:", right);
                telemetry.update();
            }

            if (gamepad1.dpad_left) {
                motorFL.setPower(left);
                motorBL.setPower(-left);
                motorFR.setPower(right);
                motorBR.setPower(-right);
            }
            else if (gamepad1.dpad_right){
                motorFL.setPower(-left);
                motorBL.setPower(left);
                motorFR.setPower(-right);
                motorBR.setPower(right);
            }
            else {
                motorFL.setPower(0);
                motorBL.setPower(0);
                motorFR.setPower(0);
                motorBR.setPower(0);
            }

            // manipulator depositing and pulling in blocks
            if (gamepad1.right_trigger > .05) {
                if (gamepad1.right_trigger < .5) {
                    manip.setPower(.25);
                }
                else if (gamepad1.right_trigger >= .5) {
                    manip.setPower(.5);
                }
                else {
                    manip.setPower(0);
                }
            }
            if (gamepad1.left_trigger > .05) {
                if (gamepad1.left_trigger < .5) {
                    manip.setPower(-.25);
                }
                else if (gamepad1.left_trigger >= .5) {
                    manip.setPower(-.5);
                }
                else {
                    manip.setPower(0);
                }
            }

            //gamepad1 end


            //gamepad2 start
            //Code for the relic extension
//            if ((Math.abs(gamepad2.right_stick_y) > .05 )){
//                relic.setPower(gamepad2.right_stick_y);
//            } else {
//                relic.setPower(0);
//
//            }
//
//        }

        // Code for the lift up and down
        if ((Math.abs(gamepad2.left_stick_y) > .05 )) {
            liftLeft.setPower(-gamepad2.left_stick_y);
            liftRight.setPower(gamepad2.left_stick_y);

        } else {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }

//        //Code for the jewel knocker start position
        if (gamepad2.y){
            jewelHand.setPower(jewelHandStart);
        }

        if (gamepad2.left_trigger >.05){
            jewelHand.setPower(-1);
        }

        if (gamepad2.right_trigger > .05){
            jewelHand.setPower(1);
        }

//
//        // grabbing and depositing the relic using servos
//        if (gamepad2.a) {
//            relicGrabber.setPosition(relicDeploy);
//        }
//        if (gamepad2.x) {
//            relicGrabber.setPosition(relicGrab);
//        }
//
//        //gamepad2 end
//
//    }

        }
    }
}

