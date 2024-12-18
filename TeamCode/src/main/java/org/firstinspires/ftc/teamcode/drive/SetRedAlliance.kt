package org.firstinspires.ftc.teamcode.drive

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.AllianceColours
import org.firstinspires.ftc.teamcode.utils.Globals

@TeleOp
class SetRedAlliance : LinearOpMode() {
    override fun runOpMode() {
        Globals.AllianceColour = AllianceColours.Red
        waitForStart();
    }
}