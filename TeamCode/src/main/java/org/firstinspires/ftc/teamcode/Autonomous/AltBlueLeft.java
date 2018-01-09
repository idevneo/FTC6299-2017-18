package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@Autonomous(name = "AltBlueLeft", group = "Sensor")
@Disabled                            // Comment this out to add to the opmode list
public class AltBlueLeft extends MyOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);
        // Set up our telemetry dashboard
        composeTelemetry();
        // Wait until we're told to go

        waitForStart();
        runtime.reset();
/**---------------------------------------------------------------------------------------------------------------*/
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        vfValue();

        jewelArm.setPosition(.55);
        jewelHand.setPosition(.4);
        sleep(500);
        jewelArm.setPosition(.15);
        sleep(1000);
        if (jewelColor.red() < jewelColor.blue()) {
            jewelHand.setPosition((.3));
        } else if (jewelColor.red() > jewelColor.blue()) {
            jewelHand.setPosition((.6));
        }
        sleep(500);

        jewelArm.setPosition(.55);
        jewelHand.setPosition(.45);
        sleep(500);
        jewelHand.setPosition(.3);
        sleep(500);

        setMotors(-.25, -.25);
        sleep(1550);
        stopMotors();


        try {
            turnCorr2(.1, 170, 5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(1000);

//        rangeMoveStrafe(3, rangeL);
//        sleep(750);

//        rangeMovePID(5.5, rangeF);
//        sleep(1000);
//        try {
//            turn(.25, 0.1);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        sleep(1000);


        rangeMoveStrafe(20, rangeL, 1);

        try {
            turnCorr2(20, 175, 2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        rangeMoveStrafe(26.75, rangeL, 1);
        sleep(350);

        vfMovePerp('b',rangeL, 1);

        manipAuto(-.75);
        sleep(500);

        manipAuto(-.75);

//back up, push forward, back up
        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();

        setMotors(.4, .4);
        sleep(250);
        stopMotors();

        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();
    }
}
