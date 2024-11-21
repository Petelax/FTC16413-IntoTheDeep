package org.firstinspires.ftc.teamcode.commands

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.groups.CommandGroup
import org.firstinspires.ftc.teamcode.subsystems.Deposit
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist

object ElevatorSample : Command {
    override val requirements: Set<Any> = setOf(Elevator, VerticalArm, VerticalWrist, Deposit)

    override val runStates: Set<Wrapper.OpModeState> = setOf(Wrapper.OpModeState.ACTIVE)

    override fun initialise() {
        TODO("Not yet implemented")
    }

    override fun execute() {
        TODO("Not yet implemented")
    }

    override fun finished(): Boolean {
        TODO("Not yet implemented")
    }

    override fun end(interrupted: Boolean) {
        TODO("Not yet implemented")
    }

    override fun toString(): String {
        return "elevator-sample"
    }


}