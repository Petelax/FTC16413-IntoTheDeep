package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.dependency.lazy.Yielding
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.utils.pathing.CurvePoint

object Telemetry: Feature {
    override var dependency: Dependency<*> = SingleAnnotation(Attach::class.java) and Yielding

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    annotation class Attach

    private var packets: MutableMap<String, Any> = mutableMapOf()

    var enabled = false

    var robotPose = Pose2d(78.0, 7.375, Rotation2d.fromDegrees(90.0))
         set(value) {
            field = value
            packets["x"] = robotPose.x
            packets["y"] = robotPose.y
            packets["heading (deg)"] = robotPose.rotation.degrees
        }


    var points: MutableSet<Pose2d> = mutableSetOf()

    var path: List<CurvePoint> = listOf()
        set(value) {
            field = value
            points = mutableSetOf()
            field.forEach{ point ->
                points.add(point.pose)
            }

        }

    var drawRobot = true
    var drawPaths = true

    override fun preUserInitHook(opMode: Wrapper) {
        opMode.opMode.telemetry.setNumDecimalPlaces(1, 5)

        path = listOf()
        points = mutableSetOf()

        val clear = TelemetryPacket()
        clear.fieldOverlay().clear()
        clear.field().clear()
        FtcDashboard.getInstance().sendTelemetryPacket(clear)

    }

    private fun update(opMode: Wrapper) {
        if (!enabled) {
            return
        }

        var packet = TelemetryPacket()
        packet.putAll(packets)

        var fieldOverlay = packet.fieldOverlay()
        fieldOverlay.setTranslation(-6 * 12.0, -6 * 12.0)
        fieldOverlay.setStroke("#3F51B5")


        if (drawPaths) {
            fieldOverlay.setAlpha(0.9)
            if (path.isNotEmpty()) {
                for (i in 0 until path.size - 1) {
                    fieldOverlay.strokeLine(
                        path[i].pose.x,
                        path[i].pose.y,
                        path[i + 1].pose.x,
                        path[i + 1].pose.y
                    )
                    //points.add(path[i].pose)
                }
                //points.add(path.last().pose)
            }

            points.forEach { pose ->
                fieldOverlay.setFill("#3F51B5").fillCircle(pose.x, pose.y, 0.5)
            }

        }

        if (drawRobot) {
            fieldOverlay.setAlpha(1.0)
            Drawing.drawRobot(fieldOverlay, Pose2d(robotPose.x, robotPose.y, robotPose.heading))
        }


        packets.forEach { (string, any) ->
            opMode.opMode.telemetry.addData(string, any)
        }

        opMode.opMode.telemetry.update()

        FtcDashboard.getInstance().sendTelemetryPacket(packet)

    }

    override fun postUserInitLoopHook(opMode: Wrapper) {
        update(opMode)

    }

    override fun postUserLoopHook(opMode: Wrapper) {
        update(opMode)

    }

    fun put(string: String, any: Any) {
        packets[string] = any

    }

    fun putCommand(string: String, any: Any): Lambda {
        return Lambda("put").setInit{put(string, any)}
    }


}