package org.firstinspires.ftc.teamcode.drive

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.AllianceColours
import org.firstinspires.ftc.teamcode.utils.Globals

@TeleOp
class setRedAlliance : OpMode() {
    override fun init() {
        Globals.AllianceColour = AllianceColours.Red
    }

    override fun loop() {

    }
}