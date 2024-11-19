package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.dashboard.config.Config

@Config
object Globals {
    @JvmField var AllianceColour = AllianceColours.Red
}

enum class AllianceColours {
    Red,
    Blue
}