package org.firstinspires.ftc.teamcode.drive

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.AllianceColours
import org.firstinspires.ftc.teamcode.utils.Globals

@TeleOp
class setBlueAlliance : OpMode() {
    override fun init() {
        Globals.AllianceColour = AllianceColours.Blue
    }

    override fun loop() {

    }
}