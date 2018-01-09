package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@Autonomous(name = "AltRedLeft", group = "Sensor")
@Disabled

public class AltRedLeft extends MyOpMode {
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

        jewelArm.setPosition(.6);
        jewelHand.setPosition(.4);
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
        sleep(1950);
        stopMotors();

        try {
            turnCorr2(.1, -70, 4000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(1000);
        vfMoveAlt();

        rangeMovePID(6.25, rangeF);
        sleep(500);

        manipAuto(-.75);
        sleep(500);

        manipAuto(-.75);

//back up, push forward, back up
        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();

        setMotors(.3, .3);
        sleep(250);
        stopMotors();

        manipAuto(-.75);
        sleep(200);

        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();
    }
}
