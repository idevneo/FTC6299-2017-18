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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@TeleOp(name="TELEOP", group="Linear Opmode")
public class Plz extends MyOpMode {

    // Declare OpMode members.
    double gamepad1_left;
    double gamepad1_right;

    double left = 1;
    double right = 1;
    double strafeMod = .25;
    double mWS = 0;
    boolean slow = false;
//    double jewelHandStart = 0.1;
//    double jewelHandDeploy = 0.9;
//    double jewelArmDeploy = .2;

    int lessenPow = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hMapT(hardwareMap);
        jewelArm.setPosition(.65);
        jewelHand.setPosition(.4);
        manipWall.setPosition(.75);

        ElapsedTime delay = new ElapsedTime();
        delay.reset();

        ElapsedTime relicDelay = new ElapsedTime();
        relicDelay.reset();

        resetStartTime();
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();

            if (!slow) {
                gamepad1_left = gamepad1.left_stick_y;
                gamepad1_right = gamepad1.right_stick_y;
            } else {
                gamepad1_left = gamepad1.left_stick_y * .35;
                gamepad1_right = gamepad1.right_stick_y* .35;
            }

            telemetry.addData("slow", slow);
            telemetry.addData("Power", gamepad1_left);

            //Movement (Gamepad 1: Left Stick, Right Stick, DPAD, b)
            //Driving forward/backwards
            if (Math.abs(gamepad1.left_stick_y) > .05 || Math.abs(gamepad1.right_stick_y) > .05) {
                motorFL.setPower(gamepad1_left);
                motorBL.setPower(gamepad1_left);
                motorFR.setPower(-gamepad1_right);
                motorBR.setPower(-gamepad1_right);
            } else if (gamepad1.dpad_left) {
                motorFL.setPower(left);
                motorBL.setPower(-left);
                motorFR.setPower(right);
                motorBR.setPower(-right);
            } else if (gamepad1.dpad_right) {
                motorFL.setPower(-left);
                motorBL.setPower(left);
                motorFR.setPower(-right);
                motorBR.setPower(right);
            } else if (gamepad1.left_bumper) {
                motorFL.setPower(-.25);
                motorBL.setPower(-.25);
                motorFR.setPower(.25);
                motorBR.setPower(.25);
            } else if (gamepad1.right_bumper) {
                motorFL.setPower(.25);
                motorBL.setPower(.25);
                motorFR.setPower(-.25);
                motorBR.setPower(-.25);
            } else if (gamepad1.b) {
                setMotors(.15, -.15); //Turns right
            } else if (gamepad1.x) {
                setMotors(-.15, .15); //Turns left
            }
            else {
                motorFL.setPower(0);
                motorBL.setPower(0);
                motorFR.setPower(0);
                motorBR.setPower(0);
            }
// Alternate Drive Train
//            if (Math.abs(gamepad1.left_stick_y) > .05 ) {
//                motorFL.setPower(gamepad1.left_stick_y);
//                motorBL.setPower(gamepad1.left_stick_y);
//                motorFR.setPower(-gamepad1.left_stick_y);
//                motorBR.setPower(-gamepad1.left_stick_y);
//            } else if (gamepad1.left_stick_x < -.05) {
//                motorFL.setPower(.8 + gamepad1.left_stick_x * -.2);
//                motorBL.setPower(-.8 + gamepad1.left_stick_x * .2);
//                motorFR.setPower(.8 + gamepad1.left_stick_x * -.2);
//                motorBR.setPower(-.8 + gamepad1.left_stick_x * .2);
//            } else if (gamepad1.left_stick_x > .05) {
//                motorFL.setPower(-.8 + gamepad1.left_stick_x * -.2);
//                motorBL.setPower(.8 + gamepad1.left_stick_x * .2);
//                motorFR.setPower(-.8 + gamepad1.left_stick_x * -.2);
//                motorBR.setPower(.8 + gamepad1.left_stick_x * .2);
//            } else if (gamepad1.right_stick_x < -.05) {
//                motorFL.setPower(-gamepad1.right_stick_x);
//                motorBL.setPower(-gamepad1.right_stick_x);
//                motorFR.setPower(-gamepad1.right_stick_x);
//                motorBR.setPower(-gamepad1.right_stick_x);
//            } else if (gamepad1.right_stick_x > .05) {
//                motorFL.setPower(-gamepad1.right_stick_x);
//                motorBL.setPower(-gamepad1.right_stick_x);
//                motorFR.setPower(-gamepad1.right_stick_x);
//                motorBR.setPower(-gamepad1.right_stick_x);
//            } else {
//                motorFL.setPower(0);
//                motorBL.setPower(0);
//                motorFR.setPower(0);
//                motorBR.setPower(0);

            //Increasing/decreasing strafing power
            if (gamepad1.dpad_up && left <= .75 && right <= .75) {
                left += strafeMod;
                right += strafeMod;
                sleep(250);
                telemetry.addData("Left:", left);
                telemetry.addData("Right:", right);
                telemetry.update();
            } else if (gamepad1.dpad_down && left >= 0.25 && right >= 0.25) {
                left -= strafeMod;
                right -= strafeMod;
                sleep(250);
                telemetry.addData("Left:", left);
                telemetry.addData("Right:", right);
                telemetry.update();
            }

            //End of Movement
            //Movement Modifiers (Gamepad 1: a,b,x,y)
            // Toggles to switch between slow mode for strafe and normal drive
            if (delay.time() > .5) {
                if (gamepad1.a && slow == false) {
                    delay.reset();
                    resetStartTime();
                    left = .25;
                    right = .25;
                    slow = true;
                } else if (gamepad1.a && slow == true) {
                    delay.reset();
                    resetStartTime();
                    left = 1;
                    right = 1;
                    slow = false;
                } else if (gamepad1.y && slow == true) {
                    delay.reset();
                    resetStartTime();
                    left = 1;
                    right = 1;
                    slow = false;
                }
            }
            //Inversion of movement
//          if (gamepad1.x) {
//              gamepad1_left *= -1;
//              gamepad1_right *= -1;
//        }
              //End of Movement Modifiers

            //Manipulator (Gamepad 1,2: Right Trigger, Left Trigger)
            if ((gamepad1.left_trigger > .15) || (gamepad2.left_trigger > .15)) {
                manip.setPower(-1);
            } else if ((gamepad1.right_trigger > .15) || (gamepad2.right_trigger > .15)) {
                manip.setPower(1);
            } else {
                manip.setPower(0);
            }

            //Manipulator Wall (Gamepad 2: x,y)
            //Deploy position for manipulator wall
            if (gamepad2.x){
                manipWall.setPosition(.035);
            }
            //Retract position for manipulator wall (init position)
            if (gamepad2.y){
                manipWall.setPosition(.75);
            }

            //Lift Movement (Gamepad 2: Left Stick)
            // Set's lift motor power to driver 2's left stick value
            if ((Math.abs(gamepad2.left_stick_y) > .05)) {
                    liftLeft.setPower(-gamepad2.left_stick_y * .5);
                    liftRight.setPower(gamepad2.left_stick_y * .5);
            }   else {
                    liftLeft.setPower(0);
                    liftRight.setPower(0);
                    liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            //Relic (Gamepad 2: Right Bumper, Left Bumper, Right Stick, a, b)
            //Open position for relic hand(with 3D printed part)
            if (gamepad2.left_bumper) {
                relicHand.setPosition(.4);
            }
            //Position for collecting the relic using the hand (with 3D print part)
            if (gamepad2.right_bumper) {
                relicHand.setPosition(0);
            }

            //Position for moving the relic wrist up or down
            if (relicDelay.milliseconds() > 75) {
                if (gamepad2.b) {
                    mWS += .05;
                    if (mWS > 1) {
                        mWS = 1;
                    }
                    relicFlip.setPosition(mWS);
                    relicDelay.reset();
                } else if (gamepad2.a) {
                    mWS -= .05;
                    if (mWS <= 0) {
                        mWS = 0;
                    }
                    relicFlip.setPosition(mWS);
                    relicDelay.reset();
                }
            }

            //When stick is pushed up the relic arm extends out
            //When stick is pushed down, the relic arm comes back in
            if ((Math.abs(gamepad2.right_stick_y) > .05)) {
                relicDrive.setPower(gamepad2.right_stick_y);

            }   else {
                relicDrive.setPower(0);
            }
         }
        }
    }






