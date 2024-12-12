package org.firstinspires.ftc.teamcode.commands

import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.groups.Race
import dev.frozenmilk.mercurial.commands.util.Wait

fun Timeout(command: Command, time: Double) : Race {
    return Race( null,
        command,
        Wait(time)
    )
}
