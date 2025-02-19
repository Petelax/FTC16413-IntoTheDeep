package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(group = "test")
class ColourTest: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var elapsedtime: ElapsedTime
    private lateinit var gamepad: GamepadEx
    //private lateinit var color: RevColorSensorV3
    //private lateinit var intake: Intake
    private lateinit var pin0: AnalogInput
    private lateinit var pin1: AnalogInput
    private lateinit var pin2: AnalogInput
    private lateinit var pin3: AnalogInput

    override fun init() {
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        //color = hardwareMap.get(RevColorSensorV3::class.java, "color")
        //intake = Intake(hardwareMap)

        pin0 = hardwareMap.analogInput.get("encoderLF")
        pin1 = hardwareMap.analogInput.get("encoderRF")
        pin2 = hardwareMap.analogInput.get("encoderLR")
        pin3 = hardwareMap.analogInput.get("encoderRR")

        gamepad = GamepadEx(gamepad1)

        elapsedtime.reset()
    }

    override fun loop() {
        // clears the cache on each hub
        for (hub in hubs) {
            hub.clearBulkCache()
        }

        //color.normalizedColors.

        //telemetry.addData("red", intake.getRed())
        //telemetry.addData("green", intake.getGreen())
        //telemetry.addData("blue", intake.getBlue())
        //telemetry.addData("distance", intake.getDistance())

        /*
        val colors = color.normalizedColors
        telemetry.addData("red", colors.red)
        telemetry.addData("blue", colors.blue)
        telemetry.addData("green", colors.green)
        telemetry.addData("alpha", colors.alpha)
        telemetry.addData("distance", color.getDistance(DistanceUnit.MM))

         */

        //telemetry.addData("state", intake.getGamePiece().name)
        telemetry.addData("0", pin0.voltage / 3.3 * 100.0)
        telemetry.addData("1", pin1.voltage / 3.3 * 100.0)
        telemetry.addData("2", pin2.voltage / 3.3 * 100.0)
        telemetry.addData("3", pin3.voltage / 3.3 * 100.0)

        telemetry.addData("ms", elapsedtime.milliseconds())
        telemetry.update()

        //intake.periodic()

        elapsedtime.reset()
    }}
