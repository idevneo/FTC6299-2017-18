package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@TeleOp(name="PSTeleop", group="Linear Opmode")
public class Post extends MyOpMode {
    // Declare OpMode members.
    double gamepadLeft;
    double gamepad1_right;
    double strafeSpeed = 1;
    double strafeMod = .25;
    double turnSpeed = .5;
    double mWS = 0;
    boolean slow = false;

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
                gamepadLeft = gamepad1.left_stick_y;
                gamepad1_right = gamepad1.right_stick_y;
            } else {
                gamepadLeft = gamepad1.left_stick_y * .35;
                gamepad1_right = gamepad1.right_stick_y* .35;
                turnSpeed = .15;
            }

            telemetry.addData("slow", slow);
            telemetry.addData("Power", gamepadLeft);

            /**Movement (Gamepad 1: Left Stick, Right Stick, DPAD, b) */
            if (Math.abs(gamepad1.left_stick_y) > .05) { //Driving forward/backwards
                motorFL.setPower(gamepadLeft);
                motorBL.setPower(gamepadLeft);
                motorFR.setPower(-gamepadLeft);
                motorBR.setPower(-gamepadLeft);
            }

            else if (gamepad1.right_stick_y < -.05) { //Turning
                double AngleTR = Math.atan((gamepad1.right_stick_x/Math.abs(gamepad1.right_stick_y)));
                telemetry.addData("AngleRS", AngleTR);
                telemetry.update();
                double AngleSpeed = AngleTR/90;
                telemetry.addData("SpeedRS", AngleSpeed);
                telemetry.update();

                if (gamepad1.right_stick_x > .05) {            //Turns Right when the stick is on right side.
                    setMotors(AngleSpeed, -AngleSpeed);
                }
                else if (gamepad1.right_stick_x < -.05) {      //Turns Left when the stick is on right side.
                    setMotors(-AngleSpeed, AngleSpeed);
                }
            }
            else if (gamepad1.right_stick_y > .05) {  //Strafing - CHANGE FOR LEFT STICK, left/right
                if (gamepad1.right_stick_x > .05) {     //Strafes Right when the stick is on right side.
                    if (Math.abs(gamepad1.right_stick_x) < .4)
                        setMotorStrafe(strafeMod);
                    else if (Math.abs(gamepad1.right_stick_x) >= .4)
                        setMotorStrafe(strafeSpeed);
                } else if (gamepad1.right_stick_x > .05) {  //Strafes left when the stick is on left side.
                    if (gamepad1.right_stick_x < -.05) {
                        if (Math.abs(gamepad1.right_stick_x) < .4)
                            setMotorStrafe(-strafeMod);
                        else if (Math.abs(gamepad1.right_stick_x) >= .4)
                            setMotorStrafe(-strafeSpeed);
                    }
                }
            }
            else if (gamepad1.left_bumper) { //Moves forwards/backwards slowly
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
                setMotors(turnSpeed, -turnSpeed); //Turns right
            } else if (gamepad1.x) {
                setMotors(-turnSpeed, turnSpeed); //Turns left
            }
            else {
                motorFL.setPower(0);
                motorBL.setPower(0);
                motorFR.setPower(0);
                motorBR.setPower(0);
            }

            //Increasing/decreasing strafing power
            if (gamepad1.dpad_up && strafeSpeed <= .75) {
                strafeSpeed += strafeMod;
                sleep(250); //To be effective would need a delay.
                telemetry.addData("Strafe Speed:", strafeSpeed);
                telemetry.update();
            } else if (gamepad1.dpad_down && strafeSpeed >= 0.25) {
                strafeSpeed -= strafeMod;
                sleep(250);
                telemetry.addData("Strafe Speed:", strafeSpeed);
                telemetry.update();
            }

            /**End of Movement
             * Movement Modifiers (Gamepad 1: a,b,x,y)
             * Toggles to switch between slow mode for strafe and normal drive */
            if (delay.time() > .5) {
                if (gamepad1.a && !slow) {
                    delay.reset();
                    resetStartTime();
                    strafeSpeed = .25;
                    slow = true;
                } else if (gamepad1.a && slow) {
                    delay.reset();
                    resetStartTime();
                    strafeSpeed = 1;
                    slow = false;
                } else if (gamepad1.y && slow) {
                    delay.reset();
                    resetStartTime();
                    strafeSpeed = 1;
                    slow = false;
                }
            }
            //Inversion of movement
//          if (gamepad1.x) {
//              gamepadLeft *= -1;
//              gamepad1_right *= -1;
//        }
              //End of Movement Modifiers

            /**Manipulator (Gamepad 1,2: Right Trigger, Left Trigger) */
            if ((gamepad1.left_trigger > .15) || (gamepad2.left_trigger > .15)) {
                manip.setPower(-1);
            } else if ((gamepad1.right_trigger > .15) || (gamepad2.right_trigger > .15)) {
                manip.setPower(1);
            } else {
                manip.setPower(0);
            }

            /**Manipulator Wall (Gamepad 2: x,y) */
            //Deploy position for manipulator wall
            if (gamepad2.x){
                manipWall.setPosition(.035);
            }
            //Retract position for manipulator wall (init position)
            if (gamepad2.y){
                manipWall.setPosition(.75);
            }

            /**Lift Movement (Gamepad 2: Left Stick) */
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

            /**Relic (Gamepad 2: Right Bumper, Left Bumper, Right Stick, a, b) */
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






