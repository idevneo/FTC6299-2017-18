package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@TeleOp(name="PSTeleop", group="Linear Opmode")
public class Post extends MyOpMode {
    // Declare OpMode members.
    double gamepadLeftY; //Forwards/Backwards Movement
    double gamepadLeftX; //Strafing Movement
    double gamepadRightX; //Turning Movement
    double gamepadRightY;

    double AngleLStick; //Left Stick Movement Control
    double AngleRStick; //Right Stick Movement Control

    double strafeSpeed = 1;
    double strafeMod = .25;
    double turnSpeed = .5; //Speed Manipulated by buttons.
    double mWS = 0;

    boolean slow = false;
    boolean align = false;

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
                gamepadLeftY = gamepad1.left_stick_y;
                gamepadLeftX = gamepad1.left_stick_x;
                gamepadRightX = gamepad1.right_stick_x;
                gamepadRightY = gamepad1.right_stick_y;
                turnSpeed = .5;
            } else {
                gamepadLeftY = gamepad1.left_stick_y * .35;
                gamepadLeftX = gamepad1.left_stick_x * .35;
                gamepadRightX = gamepad1.right_stick_x * .35;
                turnSpeed = .15;
            }

            AngleLStick = Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
            if (AngleLStick < 0) {
                AngleLStick += 360;
            }
            AngleRStick = Math.toDegrees(Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x));
            if (AngleRStick < 0) {
                AngleRStick += 360;
            }
            telemetry.addData("align", align);
            telemetry.addData("slow", slow);
            telemetry.addData("Power", gamepadLeftY);
            telemetry.update();

            /**Movement (Gamepad 1: Left Stick, Right Stick, DPAD, b) */
            if (45 <= AngleLStick && AngleLStick <= 135 || 225 <= AngleLStick && AngleLStick <= 315) { //DRIVING FORWARDS/BACKWARDS
                motorFL.setPower(gamepadLeftY); //In the future make a method which can set powers automatically.
                motorBL.setPower(gamepadLeftY);
                motorFR.setPower(-gamepadLeftY);
                motorBR.setPower(-gamepadLeftY);

                //setMotors(gamepadLeftY, gamepadLeftY);
            } else if (315 < AngleLStick && AngleLStick < 360 || 0 <= AngleLStick && AngleLStick < 45 || 135 < AngleLStick && AngleLStick < 225) { //STRAFING LEFT/RIGHT
                if (0 < gamepadLeftX && gamepadLeftX <= .2) { //Make sure the motors don't run at too low a speed.
                    gamepadLeftX = .25;
                } else if (-.2 < gamepadLeftX && gamepadLeftX < 0) { //Make sure the motors don't run at too low a speed.
                    gamepadLeftX = -.25;
                }
                setMotorStrafe(gamepadLeftX);
            }
            else if (315 < AngleLStick && AngleLStick < 360 || 0 <= AngleLStick && AngleLStick < 45) { //TURNING RIGHT
                setMotors(Math.abs(gamepadRightX), -Math.abs(gamepadRightX));
            }
            else if (225 <= AngleRStick && AngleRStick <= 315) { //TURNING LEFT
                setMotors(-Math.abs(gamepadRightX), Math.abs(gamepadRightX));
            }

            // this is to try and drive forward / backwards and turn at the same time
            else if ((315 < AngleLStick && AngleLStick < 360 || 0 <= AngleLStick && AngleLStick < 45) && (45 <= AngleLStick && AngleLStick <= 135) ) { //forwards or backwards and right turn
                motorBL.setPower(gamepadLeftY);
                motorFR.setPower(-gamepadLeftY / 2);
                motorBR.setPower(-gamepadLeftY / 2);
            }
            else if ((225 <= AngleRStick && AngleRStick <= 315) && (45 <= AngleLStick && AngleLStick <= 135) ){ // forward or backwards and left turn
                motorFL.setPower(gamepadLeftY / 2);
                motorBL.setPower(gamepadLeftY / 2);
                motorFR.setPower(-gamepadLeftY);
                motorBR.setPower(-gamepadLeftY);
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
            } else if (gamepad1.x && delay.time() > .5) { //Lines up with the crypto-box for placement of glyph.
                if (!align) {
                    align = true;
                    rangeMovePID(7, rangeF);
                }
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
//              gamepadRightY *= -1;
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






