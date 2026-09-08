package com.example.robotcontroller.network

import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener

import java.util.concurrent.TimeUnit


class RosBridgeClient(
    private val listener: Listener
) {

    interface Listener {

        fun onConnected()

        fun onDisconnected()

        fun onError(
            message: String
        )

        fun onMessage(
            message: String
        )
    }


    private val client =
        OkHttpClient
            .Builder()
            .connectTimeout(
                5,
                TimeUnit.SECONDS
            )
            .readTimeout(
                0,
                TimeUnit.MILLISECONDS
            )
            .build()


    private var webSocket:
            WebSocket? = null


    // =====================================================
    // POŁĄCZENIE
    // =====================================================

    fun connect() {

        if (webSocket != null) {
            return
        }


        val request =
            Request
                .Builder()
                .url(
                    "ws://192.168.0.94:9090"
                )
                .build()


        webSocket =
            client.newWebSocket(

                request,

                object :
                    WebSocketListener() {


                    override fun onOpen(
                        webSocket: WebSocket,
                        response: Response
                    ) {

                        advertiseCommandTopic()

                        advertisePresetTopic()

                        advertiseCartesianJogTopic()

                        advertiseTrunkLoadingTopic()

                        advertiseCubePickTopic()

                        advertiseCubePlaceTopic()

                        advertiseAutonomousDriveTopic()

                        advertiseCleanupMissionTopic()

                        subscribeRightCamera()

                        subscribeRobotStatus()

                        // Manual: aktualne kąty J1..J5.
                        subscribeJointStates()

                        // HC-SR04 NIE są już automatycznie subskrybowane.
                        // Użytkownik włącza je przyciskiem na ekranie
                        // Jazda autonomiczna.
                        subscribeAutonomousDriveStatus()

                        subscribeCleanupMissionStatus()

                        subscribeCleanupCubeCount()

                        listener.onConnected()
                    }


                    override fun onMessage(
                        webSocket: WebSocket,
                        text: String
                    ) {

                        listener.onMessage(
                            text
                        )
                    }


                    override fun onClosing(
                        webSocket: WebSocket,
                        code: Int,
                        reason: String
                    ) {

                        webSocket.close(
                            code,
                            reason
                        )
                    }


                    override fun onClosed(
                        webSocket: WebSocket,
                        code: Int,
                        reason: String
                    ) {

                        this@RosBridgeClient
                            .webSocket = null


                        listener
                            .onDisconnected()
                    }


                    override fun onFailure(
                        webSocket: WebSocket,
                        throwable: Throwable,
                        response: Response?
                    ) {

                        this@RosBridgeClient
                            .webSocket = null


                        listener.onError(
                            throwable.message
                                ?: "Nieznany błąd połączenia"
                        )
                    }
                }
            )
    }


    // =====================================================
    // ROZŁĄCZENIE
    // =====================================================


    private fun advertisePresetTopic(): Boolean {

        val json = """
        {
          "op": "advertise",
          "topic": "/manipulator/preset",
          "type": "std_msgs/msg/String"
        }
    """.trimIndent()

        return sendJson(json)
    }



    fun publishManipulatorPreset(
        preset: String
    ): Boolean {

        val json = """
        {
          "op": "publish",
          "topic": "/manipulator/preset",
          "msg": {
            "data": "$preset"
          }
        }
    """.trimIndent()

        return sendJson(json)
    }


    fun disconnect() {

        unsubscribeRightCamera()

        unsubscribeStereoCamera()

        unsubscribeDepthCamera()

        unsubscribeDepthCenterDistance()

        unsubscribeAiCamera()

        // Dane dokładnie z toru ArUco używanego przez cube_pick.
        unsubscribeArucoDebugCamera()
        unsubscribeArucoPosition()
        unsubscribeArucoDetected()
        unsubscribeArucoId()
        unsubscribeArucoYaw()
        unsubscribeArucoQuality()
        unsubscribeCubePickStatus()
        unsubscribeCubePlaceStatus()
        unsubscribeTrunkLoadingStatus()

        unsubscribeRobotStatus()

        unsubscribeJointStates()

        unsubscribeUltrasonic()

        unsubscribeAutonomousDriveStatus()

        unsubscribeCleanupMissionStatus()

        unsubscribeCleanupCubeCount()


        webSocket?.close(
            1000,
            "Rozłączenie przez użytkownika"
        )


        webSocket = null
    }


    // =====================================================
    // DOTYCHCZASOWE KOMENDY MANIPULATORA
    // =====================================================

    private fun advertiseCommandTopic():
            Boolean {

        val json = """
            {
              "op": "advertise",
              "topic": "/manipulator/command",
              "type": "std_msgs/msg/String"
            }
        """.trimIndent()


        return sendJson(
            json
        )
    }


    fun publishManipulatorCommand(
        command: String
    ): Boolean {

        val json = """
            {
              "op": "publish",
              "topic": "/manipulator/command",
              "msg": {
                "data": "$command"
              }
            }
        """.trimIndent()


        return sendJson(
            json
        )
    }


    // =====================================================
    // KINEMATYKA ODWROTNA - RUCH WZGLEDNY TCP
    //
    // std_msgs/msg/String na /manipulator/cartesian_jog
    // Przyklady:
    //   x+  x-  y+  y-  z+  z-
    //   step:0.005
    //   status
    // =====================================================

    private fun advertiseCartesianJogTopic():
            Boolean {

        val json = """
            {
              "op": "advertise",
              "topic": "/manipulator/cartesian_jog",
              "type": "std_msgs/msg/String"
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun publishCartesianJog(
        command: String
    ): Boolean {

        val safeCommand =
            command
                .trim()
                .lowercase()


        val json = """
            {
              "op": "publish",
              "topic": "/manipulator/cartesian_jog",
              "msg": {
                "data": "$safeCommand"
              }
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    // =====================================================
    // CUBE PICK / CUBE PLACE - osobny ekran "Podniesienie kostki"
    // =====================================================

    private fun advertiseCubePickTopic(): Boolean {

        val json = """
            {
              "op": "advertise",
              "topic": "/cube_pick/command",
              "type": "std_msgs/msg/String"
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun publishCubePickCommand(
        command: String
    ): Boolean {

        val safeCommand = command.trim().lowercase()

        val json = """
            {
              "op": "publish",
              "topic": "/cube_pick/command",
              "msg": {
                "data": "$safeCommand"
              }
            }
        """.trimIndent()

        return sendJson(json)
    }


    private fun advertiseCubePlaceTopic(): Boolean {

        val json = """
            {
              "op": "advertise",
              "topic": "/cube_place/command",
              "type": "std_msgs/msg/String"
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun publishCubePlaceCommand(
        command: String
    ): Boolean {

        val safeCommand = command.trim().lowercase()

        val json = """
            {
              "op": "publish",
              "topic": "/cube_place/command",
              "msg": {
                "data": "$safeCommand"
              }
            }
        """.trimIndent()

        return sendJson(json)
    }


    // =====================================================
    // PELNY ZALADUNEK DO BAGAZNIKA
    //
    // Telefon uruchamia tylko jeden topic:
    // /trunk_loading/command -> "start"
    //
    // Raspberry Pi robi dalej:
    // cube_pick full -> cube_place start.
    // =====================================================

    private fun advertiseTrunkLoadingTopic():
            Boolean {

        val json = """
            {
              "op": "advertise",
              "topic": "/trunk_loading/command",
              "type": "std_msgs/msg/String"
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun publishTrunkLoadingCommand(
        command: String
    ): Boolean {

        val safeCommand =
            command
                .trim()
                .lowercase()

        val json = """
            {
              "op": "publish",
              "topic": "/trunk_loading/command",
              "msg": {
                "data": "$safeCommand"
              }
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    // =====================================================
    // JAZDA AUTONOMICZNA
    // =====================================================

    private fun advertiseAutonomousDriveTopic():
            Boolean {

        val json = """
            {
              "op": "advertise",
              "topic": "/autonomous_drive/command",
              "type": "std_msgs/msg/String"
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun publishAutonomousDriveCommand(
        command: String
    ): Boolean {

        val safeCommand =
            command
                .trim()
                .lowercase()

        val json = """
            {
              "op": "publish",
              "topic": "/autonomous_drive/command",
              "msg": {
                "data": "$safeCommand"
              }
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    // =====================================================
    // AUTONOMICZNE ZBIERANIE
    // =====================================================

    private fun advertiseCleanupMissionTopic():
            Boolean {

        val json = """
            {
              "op": "advertise",
              "topic": "/cleanup_mission/command",
              "type": "std_msgs/msg/String"
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun publishCleanupMissionCommand(
        command: String
    ): Boolean {

        val safeCommand =
            command
                .trim()
                .lowercase()

        val json = """
            {
              "op": "publish",
              "topic": "/cleanup_mission/command",
              "msg": {
                "data": "$safeCommand"
              }
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun subscribeCleanupMissionStatus():
            Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_cleanup_status",
              "topic": "/cleanup_mission/status",
              "type": "std_msgs/msg/String",
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun unsubscribeCleanupMissionStatus():
            Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_cleanup_status",
              "topic": "/cleanup_mission/status"
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun subscribeCleanupCubeCount():
            Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_cleanup_cube_count",
              "topic": "/cleanup_mission/cubes_loaded",
              "type": "std_msgs/msg/Int32",
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun unsubscribeCleanupCubeCount():
            Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_cleanup_cube_count",
              "topic": "/cleanup_mission/cubes_loaded"
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    // =====================================================
    // PRAWA KAMERA
    // =====================================================

    fun subscribeRightCamera():
            Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_right_camera",
              "topic": "/camera_right/right_camera/image_raw/compressed",
              "type": "sensor_msgs/msg/CompressedImage",
              "throttle_rate": 400,
              "queue_length": 1
            }
        """.trimIndent()


        return sendJson(
            json
        )
    }


    fun unsubscribeRightCamera():
            Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_right_camera",
              "topic": "/camera_right/right_camera/image_raw/compressed"
            }
        """.trimIndent()


        return sendJson(
            json
        )
    }


    // =====================================================
    // STEREO
    // =====================================================

    fun subscribeStereoCamera():
            Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_stereo_camera",
              "topic": "/stereo/rectified_check/compressed",
              "type": "sensor_msgs/msg/CompressedImage",
              "throttle_rate": 200,
              "queue_length": 1
            }
        """.trimIndent()


        return sendJson(
            json
        )
    }


    fun unsubscribeStereoCamera():
            Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_stereo_camera",
              "topic": "/stereo/rectified_check/compressed"
            }
        """.trimIndent()


        return sendJson(
            json
        )
    }


    // =====================================================
    // MAPA GŁĘBI
    // =====================================================

    fun subscribeDepthCamera():
            Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_depth_camera",
              "topic": "/stereo/depth/compressed",
              "type": "sensor_msgs/msg/CompressedImage",
              "throttle_rate": 200,
              "queue_length": 1
            }
        """.trimIndent()


        return sendJson(
            json
        )
    }


    fun unsubscribeDepthCamera():
            Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_depth_camera",
              "topic": "/stereo/depth/compressed"
            }
        """.trimIndent()


        return sendJson(
            json
        )
    }


    // =====================================================
    // ODLEGŁOŚĆ Z MAPY GŁĘBI - ŚRODEK OBRAZU
    // =====================================================

    fun subscribeDepthCenterDistance(): Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_depth_center_distance",
              "topic": "/stereo/center_distance",
              "type": "std_msgs/msg/Float32",
              "throttle_rate": 100,
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun unsubscribeDepthCenterDistance(): Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_depth_center_distance",
              "topic": "/stereo/center_distance"
            }
        """.trimIndent()

        return sendJson(json)
    }


    // =====================================================
    // AI - POZYCJA KOSTKI / ODLEGLOSC Z
    // =====================================================

    fun subscribeAiCubePosition():
            Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_ai_cube_position",
              "topic": "/ai_cube/position_camera",
              "type": "geometry_msgs/msg/PointStamped",
              "throttle_rate": 100,
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun unsubscribeAiCubePosition():
            Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_ai_cube_position",
              "topic": "/ai_cube/position_camera"
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun subscribeAiFastPosition(): Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_cleanup_ai_fast_position",
              "topic": "/ai_cube/position_camera_fast",
              "type": "geometry_msgs/msg/PointStamped",
              "throttle_rate": 60,
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun unsubscribeAiFastPosition(): Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_cleanup_ai_fast_position",
              "topic": "/ai_cube/position_camera_fast"
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun subscribeAiDetected(): Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_cleanup_ai_detected",
              "topic": "/ai_cube/detected",
              "type": "std_msgs/msg/Bool",
              "throttle_rate": 80,
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun unsubscribeAiDetected(): Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_cleanup_ai_detected",
              "topic": "/ai_cube/detected"
            }
        """.trimIndent()

        return sendJson(json)
    }


    // =====================================================
    // AI - WYKRYWANIE KOSTKI
    // =====================================================

    fun subscribeAiCamera():
            Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_ai_cube_camera",
              "topic": "/ai_cube/debug/compressed",
              "type": "sensor_msgs/msg/CompressedImage",
              "throttle_rate": 120,
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun unsubscribeAiCamera():
            Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_ai_cube_camera",
              "topic": "/ai_cube/debug/compressed"
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    // =====================================================
    // CLEANUP / CUBE PICK - DANE ARUCO
    //
    // To są DOKŁADNIE topiki publikowane przez aruco_cube_detector
    // i używane później przez cube_pick_node.
    // =====================================================

    fun subscribeArucoDebugCamera(): Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_cleanup_aruco_debug",
              "topic": "/cube_aruco/debug/compressed",
              "type": "sensor_msgs/msg/CompressedImage",
              "throttle_rate": 120,
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun unsubscribeArucoDebugCamera(): Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_cleanup_aruco_debug",
              "topic": "/cube_aruco/debug/compressed"
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun subscribeArucoPosition(): Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_cleanup_aruco_position",
              "topic": "/cube_aruco/position_camera",
              "type": "geometry_msgs/msg/PointStamped",
              "throttle_rate": 80,
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun unsubscribeArucoPosition(): Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_cleanup_aruco_position",
              "topic": "/cube_aruco/position_camera"
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun subscribeArucoDetected(): Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_cleanup_aruco_detected",
              "topic": "/cube_aruco/detected",
              "type": "std_msgs/msg/Bool",
              "throttle_rate": 100,
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun unsubscribeArucoDetected(): Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_cleanup_aruco_detected",
              "topic": "/cube_aruco/detected"
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun subscribeArucoId(): Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_cleanup_aruco_id",
              "topic": "/cube_aruco/id",
              "type": "std_msgs/msg/Int32",
              "throttle_rate": 100,
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun unsubscribeArucoId(): Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_cleanup_aruco_id",
              "topic": "/cube_aruco/id"
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun subscribeArucoYaw(): Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_cleanup_aruco_yaw",
              "topic": "/cube_aruco/yaw_deg",
              "type": "std_msgs/msg/Float32",
              "throttle_rate": 100,
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun unsubscribeArucoYaw(): Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_cleanup_aruco_yaw",
              "topic": "/cube_aruco/yaw_deg"
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun subscribeArucoQuality(): Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_cleanup_aruco_quality",
              "topic": "/cube_aruco/quality",
              "type": "std_msgs/msg/Float32",
              "throttle_rate": 100,
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun unsubscribeArucoQuality(): Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_cleanup_aruco_quality",
              "topic": "/cube_aruco/quality"
            }
        """.trimIndent()

        return sendJson(json)
    }


    // =====================================================
    // STATUSY GOTOWYCH NODÓW PICK / PLACE / TRUNK
    // =====================================================

    fun subscribeCubePickStatus(): Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_cube_pick_status",
              "topic": "/cube_pick/status",
              "type": "std_msgs/msg/String",
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun unsubscribeCubePickStatus(): Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_cube_pick_status",
              "topic": "/cube_pick/status"
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun subscribeCubePlaceStatus(): Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_cube_place_status",
              "topic": "/cube_place/status",
              "type": "std_msgs/msg/String",
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun unsubscribeCubePlaceStatus(): Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_cube_place_status",
              "topic": "/cube_place/status"
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun subscribeTrunkLoadingStatus(): Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_trunk_loading_status",
              "topic": "/trunk_loading/status",
              "type": "std_msgs/msg/String",
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(json)
    }


    fun unsubscribeTrunkLoadingStatus(): Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_trunk_loading_status",
              "topic": "/trunk_loading/status"
            }
        """.trimIndent()

        return sendJson(json)
    }


    // =====================================================
    // STATUS ROBOTA
    // =====================================================

    // =====================================================
    // MANUAL - AKTUALNE JOINT STATES
    // =====================================================

    fun subscribeJointStates():
            Boolean {

        val json = """
            {
              "op": "subscribe",
              "topic": "/joint_states",
              "type": "sensor_msgs/msg/JointState",
              "throttle_rate": 100
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun unsubscribeJointStates():
            Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "topic": "/joint_states"
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun subscribeRobotStatus():
            Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_robot_status",
              "topic": "/robot/status",
              "type": "std_msgs/msg/String",
              "queue_length": 1
            }
        """.trimIndent()


        return sendJson(
            json
        )
    }


    fun unsubscribeRobotStatus():
            Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_robot_status",
              "topic": "/robot/status"
            }
        """.trimIndent()


        return sendJson(
            json
        )
    }


    // =====================================================
    // HC-SR04
    // =====================================================

    fun subscribeUltrasonic():
            Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_ultrasonic",
              "topic": "/robot/ultrasonic",
              "type": "std_msgs/msg/String",
              "throttle_rate": 100,
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun unsubscribeUltrasonic():
            Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_ultrasonic",
              "topic": "/robot/ultrasonic"
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    // =====================================================
    // STATUS JAZDY AUTONOMICZNEJ
    // =====================================================

    fun subscribeAutonomousDriveStatus():
            Boolean {

        val json = """
            {
              "op": "subscribe",
              "id": "android_autonomous_drive_status",
              "topic": "/autonomous_drive/status",
              "type": "std_msgs/msg/String",
              "queue_length": 1
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    fun unsubscribeAutonomousDriveStatus():
            Boolean {

        val json = """
            {
              "op": "unsubscribe",
              "id": "android_autonomous_drive_status",
              "topic": "/autonomous_drive/status"
            }
        """.trimIndent()

        return sendJson(
            json
        )
    }


    // =====================================================
    // TEST
    // =====================================================

    fun publishTestMessage():
            Boolean {

        val json = """
            {
              "op": "publish",
              "topic": "/mobile_test",
              "msg": {
                "data": "Wiadomosc z telefonu"
              }
            }
        """.trimIndent()


        return sendJson(
            json
        )
    }


    // =====================================================
    // JSON
    // =====================================================

    private fun sendJson(
        json: String
    ): Boolean {

        return webSocket
            ?.send(
                json
            )
            ?: false
    }


    // =====================================================
    // SHUTDOWN
    // =====================================================

    fun shutdown() {

        unsubscribeRightCamera()

        unsubscribeStereoCamera()

        unsubscribeDepthCamera()

        unsubscribeAiCamera()

        unsubscribeRobotStatus()

        unsubscribeUltrasonic()

        unsubscribeAutonomousDriveStatus()

        unsubscribeCleanupMissionStatus()

        unsubscribeCleanupCubeCount()


        webSocket?.close(
            1000,
            "Zamknięcie aplikacji"
        )


        webSocket = null


        client.dispatcher
            .executorService
            .shutdown()


        client.connectionPool
            .evictAll()
    }
}