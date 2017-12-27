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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Library.MyOpMode;


@Autonomous(name="RedRightDriveLeft", group="Linear Opmode")

public class AutoRedDriveLeft extends MyOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);

        waitForStart();
        runtime.reset();

//        while(opModeIsActive()) {
//            telemetry.addData("Red  ", jewelColor.red());
//            telemetry.addData("Green", jewelColor.green());
//            telemetry.addData("Blue ", jewelColor.blue());
//            telemetry.update();
//        }

        // run until the end of the match (driver presses STOP)
//            jewelArm.setPosition(.6);
//            jewelHand.setPosition(.45);
//            sleep(2000);
//            jewelArm.setPosition(.15);
//            sleep(2000);
//
//            if (jewelColor.red() > jewelColor.blue()) {
//                jewelHand.setPosition((.3));
//
//            } else if (jewelColor.red() < jewelColor.blue()) {
//                jewelHand.setPosition((.6));
//            }
//
//            sleep(1000);
//            jewelArm.setPosition(.6);
//            jewelHand.setPosition(.45);
//            sleep(3000);
//
//            jewelHand.setPosition(.3);
//            sleep(1000);
//
//            manip.setPower(1);
//            Thread.sleep(1000);
//
//            motorFL.setPower(-0.5);
//            motorBL.setPower(-0.5);
//            motorFR.setPower(0.5);
//            motorBR.setPower(0.5);
//            Thread.sleep(1500);
//
//            motorFL.setPower(0);
//            motorBL.setPower(0);
//            motorFR.setPower(0);
//            motorBR.setPower(0);
//
        liftLeft.setPower(.5);
        liftRight.setPower(-.5);
        sleep(250);
        liftLeft.setPower(0);
        liftRight.setPower(0);


        jewelArm.setPosition(.6);
        jewelHand.setPosition(.45);
        sleep(750);
        jewelArm.setPosition(.15);
        sleep(1000);
        if (jewelColor.red() > jewelColor.blue()) {
            jewelHand.setPosition((.3));
        } else if (jewelColor.red() < jewelColor.blue()) {
            jewelHand.setPosition((.6));
        }
        sleep(500);

        jewelArm.setPosition(.6);
        jewelHand.setPosition(.45);
        sleep(1000);
        jewelHand.setPosition(.3);
        sleep(500);

        setMotors(.25,.25);
        sleep(500);
        stopMotors();

        rangeMovePID(9, rangeF);
        sleep(1000);
//        try {
//            turn(.25, 0.1);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        sleep(1000);

        rangeMoveStrafe(26.25, rangeR, 0);

//[
//        try {
//            turnCorr(.25, -0.5, 3000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        sleep(1000);

//        vfMovePerp( 'r', rangeR);

        rangeMovePID(7, rangeF);
        sleep(500);

        manipAuto(-.75);
        sleep(500);

        manipAuto(-.75);

//back up, push forward, back up
        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();

        setMotors(-.4, -.4);
        sleep(250);
        stopMotors();

        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();
        //Finish optimizing this Auto, then invert for the blue side.
        // Loop and update the dashboard
//        while (opModeIsActive()) {
//
//            telemetry.update();
//        }
    }

}


