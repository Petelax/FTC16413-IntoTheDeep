package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Intake

class IntakeRelease(private var intake: Intake): SequentialCommandGroup() {
    init {
        addRequirements(intake)
        addCommands(
            InstantCommand({intake.setSpeed(HorizontalConstants.IntakeSpeeds.MAX)}),
            WaitCommand(300),
            InstantCommand({intake.setSpeed(0.0)})
        )
    }
}