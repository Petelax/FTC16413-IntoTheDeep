package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension

@TeleOp(group = "test")
class ElevatorTest: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var elapsedtime: ElapsedTime
    private lateinit var gamepad: GamepadEx
    private lateinit var elevator: HorizontalExtension
    private lateinit var motor: DcMotor

    override fun init() {
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        //voltage = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()

        elevator = HorizontalExtension(hardwareMap)
        //motor = hardwareMap.dcMotor.get("elevatorLeft")

        gamepad = GamepadEx(gamepad1)

        elapsedtime.reset()
    }

    override fun loop() {

        elevator.setSpeed(gamepad.leftY)

        telemetry.addData("speed", gamepad.leftY)
        //telemetry.addData("raw position", elevator.getRawPosition())
        telemetry.addData("position", elevator.getPosition())
        telemetry.addData("current", elevator.getCurrent())

        elevator.periodic()

        telemetry.update()
    }
}