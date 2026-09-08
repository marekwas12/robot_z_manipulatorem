package com.example.robotcontroller.viewmodel

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.os.Handler
import android.os.Looper
import android.util.Base64

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue

import androidx.lifecycle.ViewModel

import com.example.robotcontroller.network.RosBridgeClient

import org.json.JSONObject

import java.util.Locale

import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin


class MainViewModel :
    ViewModel(),
    RosBridgeClient.Listener {


    enum class ConnectionState {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        ERROR
    }


    // =====================================================
    // STAN POŁĄCZENIA
    // =====================================================

    var connectionState by mutableStateOf(
        ConnectionState.DISCONNECTED
    )
        private set


    var connectionMessage by mutableStateOf(
        "Rozłączony"
    )
        private set


    var lastRosMessage by mutableStateOf("")
        private set


    val isConnected: Boolean
        get() =
            connectionState ==
                    ConnectionState.CONNECTED


    val isConnecting: Boolean
        get() =
            connectionState ==
                    ConnectionState.CONNECTING


    // =====================================================
    // PRAWA KAMERA
    //
    // Podgląd trybu manualnego.
    // =====================================================

    var cameraBitmap by mutableStateOf<Bitmap?>(
        null
    )
        private set


    @Volatile
    private var isDecodingCameraFrame =
        false


    // =====================================================
    // TRYB MANUALNY - AKTUALNE JOINTY + TCP
    // =====================================================

    var jointPositionsRad by mutableStateOf<List<Double?>>(
        List(
            5
        ) {
            null
        }
    )
        private set


    val jointAnglesDeg: List<Double?>
        get() =
            jointPositionsRad.map {
                    value ->

                value
                    ?.let {
                        Math.toDegrees(
                            it
                        )
                    }
            }


    var tcpXmm by mutableStateOf<Double?>(
        null
    )
        private set


    var tcpYmm by mutableStateOf<Double?>(
        null
    )
        private set


    var tcpZmm by mutableStateOf<Double?>(
        null
    )
        private set


    // =====================================================
    // STEREO
    // =====================================================

    var stereoBitmap by mutableStateOf<Bitmap?>(
        null
    )
        private set


    var stereoEnabled by mutableStateOf(
        false
    )
        private set


    @Volatile
    private var isDecodingStereoFrame =
        false


    // =====================================================
    // MAPA GŁĘBI
    // =====================================================

    var depthBitmap by mutableStateOf<Bitmap?>(
        null
    )
        private set


    var depthEnabled by mutableStateOf(
        false
    )
        private set


    // Odległość najbliższego/centralnego obiektu z /stereo/center_distance.
    var depthCenterDistanceM by mutableStateOf<Float?>(
        null
    )
        private set


    @Volatile
    private var isDecodingDepthFrame =
        false


    // =====================================================
    // AI - WYKRYWANIE KOSTKI
    // =====================================================

    var aiBitmap by mutableStateOf<Bitmap?>(
        null
    )
        private set


    var aiEnabled by mutableStateOf(
        false
    )
        private set


    @Volatile
    private var isDecodingAiFrame =
        false


    // =====================================================
    // CLEANUP - ARUCO DEBUG
    //
    // To jest /cube_aruco/debug/compressed, czyli obraz z TEGO
    // detektora, którego używa później cube_pick_node.
    // =====================================================

    var arucoBitmap by mutableStateOf<Bitmap?>(
        null
    )
        private set

    var arucoEnabled by mutableStateOf(
        false
    )
        private set

    @Volatile
    private var isDecodingArucoFrame =
        false


    // =====================================================
    // TEMPERATURY I URZĄDZENIA
    // =====================================================

    var upperTemperature by mutableStateOf<Float?>(
        null
    )
        private set


    var lowerTemperature by mutableStateOf<Float?>(
        null
    )
        private set


    var upperHumidity by mutableStateOf<Float?>(
        null
    )
        private set


    var lowerHumidity by mutableStateOf<Float?>(
        null
    )
        private set


    var upperFanEnabled by mutableStateOf(
        false
    )
        private set


    var lowerFanEnabled by mutableStateOf(
        false
    )
        private set


    var buzzerEnabled by mutableStateOf(
        false
    )
        private set


    // =====================================================
    // JAZDA AUTONOMICZNA / HC-SR04
    // =====================================================

    var ultrasonicFrontLeft by mutableStateOf<Float?>(
        null
    )
        private set

    var ultrasonicFrontCenter by mutableStateOf<Float?>(
        null
    )
        private set

    var ultrasonicFrontRight by mutableStateOf<Float?>(
        null
    )
        private set

    var ultrasonicLeft by mutableStateOf<Float?>(
        null
    )
        private set

    var ultrasonicRight by mutableStateOf<Float?>(
        null
    )
        private set

    var ultrasonicRear by mutableStateOf<Float?>(
        null
    )
        private set

    // Czy telefon aktualnie subskrybuje /robot/ultrasonic.
    // Czujniki mogą być oglądane także przy wyłączonej jeździe.
    var ultrasonicEnabled by mutableStateOf(
        false
    )
        private set

    var autonomousDriveStatus by mutableStateOf(
        "STOP"
    )
        private set


    // =====================================================
    // AUTONOMICZNE ZBIERANIE
    // =====================================================

    var cleanupMissionStatus by mutableStateOf(
        "STOP"
    )
        private set

    var cubesLoaded by mutableStateOf(
        0
    )
        private set


    // =====================================================
    // CLEANUP DRIVE - AI
    // =====================================================

    var aiDriveDetected by mutableStateOf(false)
        private set

    var aiDriveX by mutableStateOf<Float?>(null)
        private set

    var aiDriveY by mutableStateOf<Float?>(null)
        private set

    var aiDriveDistanceM by mutableStateOf<Float?>(null)
        private set

    var aiDriveBearingDeg by mutableStateOf<Float?>(null)
        private set


    // =====================================================
    // CLEANUP - DANE Z DOKŁADNIE TYCH SAMYCH TOPIKÓW,
    // KTÓRE CZYTA cube_pick_node.
    // =====================================================

    var arucoDetected by mutableStateOf(
        false
    )
        private set

    var arucoCubeId by mutableStateOf<Int?>(
        null
    )
        private set

    var arucoCubeX by mutableStateOf<Float?>(
        null
    )
        private set

    var arucoCubeY by mutableStateOf<Float?>(
        null
    )
        private set

    // To jest DOKŁADNIE point.z z /cube_aruco/position_camera.
    var arucoCubeDistanceM by mutableStateOf<Float?>(
        null
    )
        private set

    // Kąt celu względem środka kamery do sterowania platformą.
    // X/Z nadal są dokładnie z /cube_aruco/position_camera.
    //
    // Do LEWO/PRAWO platformy używamy jednak:
    // atan2(-X, Z),
    // ponieważ fizyczna kamera jest obrócona o 180°.
    //
    // Surowe X/Y/Z pokazywane na telefonie NIE są zmieniane.
    //
    // UWAGA: to NIE jest yaw kostki.
    var arucoBearingDeg by mutableStateOf<Float?>(
        null
    )
        private set

    // To jest prawdziwy /cube_aruco/yaw_deg używany przez cube_pick
    // do ustawienia J5.
    var arucoCubeYawDeg by mutableStateOf<Float?>(
        null
    )
        private set

    var arucoQuality by mutableStateOf<Float?>(
        null
    )
        private set

    // Statusy gotowych node'ów.
    var cubePickStatus by mutableStateOf(
        "idle"
    )
        private set

    var cubePlaceStatus by mutableStateOf(
        "idle"
    )
        private set

    var trunkLoadingStatus by mutableStateOf(
        "idle"
    )
        private set


    // =====================================================
    // MANUAL JOINT JOG / FK
    // =====================================================

    private val jointStepRad =
        Math.toRadians(
            1.0
        )


    private val stepsPerRad =
        16640.0 / (2.0 * PI)


    private val jointMinRad =
        doubleArrayOf(
            -5.87425,
            0.0,
            0.0,
            Math.toRadians(
                5.0
            ),
            // J5 - tylko sterowanie przegubowe MANUALNE:
            // stary limit 52°, rozszerzony o 15° w dół.
            Math.toRadians(
                37.0
            )
        )


    private val jointMaxRad =
        doubleArrayOf(
            0.0,
            3.81371,
            4.46884,
            PI,
            // J5 - tylko sterowanie przegubowe MANUALNE:
            // stary limit 112°, rozszerzony o 15° w górę.
            Math.toRadians(
                127.0
            )
        )


    // Ten sam model co robot_motion/arm_kinematics.py.
    private val j1J2M =
        0.155

    private val j2J3M =
        0.295

    private val j3J4SideM =
        0.010

    private val j3J4M =
        0.150

    private val j4J5M =
        0.115

    private val j5TcpM =
        0.140

    private val j2StraightRad =
        1.502451582768484

    private val j3StraightRad =
        2.2172394305143346

    private val j4StraightRad =
        1.6580627893946132


    // =====================================================
    // NARZĘDZIA
    // =====================================================

    private val mainHandler =
        Handler(
            Looper.getMainLooper()
        )


    private val rosBridgeClient =
        RosBridgeClient(
            this
        )


    // =====================================================
    // POŁĄCZENIE
    // =====================================================


    fun sendManipulatorPreset(
        preset: String
    ) {

        if (!isConnected) {

            connectionMessage =
                "Brak połączenia z robotem"

            return
        }


        val sent =
            rosBridgeClient
                .publishManipulatorPreset(
                    preset
                )


        connectionMessage =
            if (sent) {

                "Preset: ${preset.uppercase()}"

            } else {

                "Nie udało się wysłać presetu"
            }
    }



    fun connect() {

        if (
            connectionState ==
            ConnectionState.CONNECTING
            ||
            connectionState ==
            ConnectionState.CONNECTED
        ) {

            return
        }


        connectionState =
            ConnectionState.CONNECTING


        connectionMessage =
            "Łączenie z robotem..."


        rosBridgeClient.connect()
    }



    fun jogManualJoint(
        jointIndex: Int,
        deltaDegrees: Int
    ) {

        if (!isConnected) {
            connectionMessage =
                "Brak połączenia z robotem"
            return
        }

        if (jointIndex !in 0..4) {
            return
        }

        if (deltaDegrees !in listOf(-10, -5, -1, 1, 5, 10)) {
            connectionMessage =
                "Niepoprawny krok osi"
            return
        }

        val current = jointPositionsRad

        if (current.any { it == null }) {
            connectionMessage =
                "Brak aktualnych /joint_states"
            return
        }

        val q = current.map { it!! }.toMutableList()

        // Jeden przycisk = jeden rzeczywisty target.
        // +10 wysyła target oddalony dokładnie o 10 stopni
        // od aktualnego /joint_states, zamiast 10 razy wysyłać +1
        // na podstawie tej samej, jeszcze nieodświeżonej pozycji.
        val requested =
            q[jointIndex] +
                    Math.toRadians(
                        deltaDegrees.toDouble()
                    )

        q[jointIndex] = requested.coerceIn(
            jointMinRad[jointIndex],
            jointMaxRad[jointIndex]
        )

        val j1Steps =
            (q[0] * stepsPerRad).roundToInt()

        val j2Steps =
            (q[1] * stepsPerRad).roundToInt()

        val j3Steps =
            (q[2] * stepsPerRad).roundToInt()

        val j4Deg = Math.toDegrees(q[3])
        val j5Deg = Math.toDegrees(q[4])

        val command = String.format(
            Locale.US,
            "@J:%d,%d,%d,%.1f,%.1f",
            j1Steps,
            j2Steps,
            j3Steps,
            j4Deg,
            j5Deg
        )

        val sent =
            rosBridgeClient.publishManipulatorCommand(command)

        connectionMessage =
            if (sent) {
                "Oś ${jointIndex + 1}: " +
                        String.format(
                            Locale.US,
                            "%+.1f°",
                            Math.toDegrees(q[jointIndex])
                        )
            } else {
                "Nie udało się wysłać ruchu osi"
            }
    }


    fun sendIkJog(
        command: String
    ) {

        if (!isConnected) {

            connectionMessage =
                "Brak połączenia z robotem"

            return
        }


        val allowed = setOf(
            "x+",
            "x-",
            "y+",
            "y-",
            "z+",
            "z-",
            "status"
        )

        val normalized =
            command
                .trim()
                .lowercase()

        val valid =
            normalized in allowed
                    ||
                    normalized.startsWith(
                        "step:"
                    )

        if (!valid) {

            connectionMessage =
                "Niepoprawna komenda IK"

            return
        }


        val sent =
            rosBridgeClient
                .publishCartesianJog(
                    normalized
                )


        connectionMessage =
            if (sent) {

                when (normalized) {
                    "x+" -> "IK: X +5 mm"
                    "x-" -> "IK: X -5 mm"
                    "y+" -> "IK: Y +5 mm"
                    "y-" -> "IK: Y -5 mm"
                    "z+" -> "IK: Z +5 mm"
                    "z-" -> "IK: Z -5 mm"
                    "status" -> "Żądanie pozycji TCP"
                    else -> "IK: $normalized"
                }

            } else {

                "Nie udało się wysłać jog IK"
            }
    }



    fun disconnect() {

        stopAutonomousPreview()

        rosBridgeClient.disconnect()


        connectionState =
            ConnectionState.DISCONNECTED


        connectionMessage =
            "Rozłączony"


        cameraBitmap = null

        stereoBitmap = null
        stereoEnabled = false

        depthBitmap = null
        depthEnabled = false
        depthCenterDistanceM = null

        aiBitmap = null
        aiEnabled = false


        ultrasonicFrontLeft = null
        ultrasonicFrontCenter = null
        ultrasonicFrontRight = null
        ultrasonicLeft = null
        ultrasonicRight = null
        ultrasonicRear = null
        ultrasonicEnabled = false

        autonomousDriveStatus =
            "STOP"

        cleanupMissionStatus =
            "STOP"

        cubesLoaded = 0


        upperTemperature = null
        lowerTemperature = null

        upperHumidity = null
        lowerHumidity = null


        upperFanEnabled = false
        lowerFanEnabled = false

        buzzerEnabled = false
    }


    fun toggleConnection() {

        if (isConnected) {

            disconnect()

        } else {

            connect()
        }
    }


    // =====================================================
    // STEREO ON / OFF
    // =====================================================

    fun toggleStereoPreview() {

        if (!isConnected) {

            connectionMessage =
                "Brak połączenia z robotem"

            return
        }


        // Jeśli już działa -> wyłącz
        if (stereoEnabled) {

            rosBridgeClient
                .unsubscribeStereoCamera()

            stereoEnabled = false

            stereoBitmap = null

            return
        }


        // Jeśli działa mapa głębi,
        // wyłączamy ją przed stereo.
        if (depthEnabled) {

            rosBridgeClient
                .unsubscribeDepthCamera()

            depthEnabled = false

            depthBitmap = null
        }


        if (aiEnabled) {

            rosBridgeClient
                .unsubscribeAiCamera()

            aiEnabled = false
            aiBitmap = null
        }


        val success =
            rosBridgeClient
                .subscribeStereoCamera()


        if (success) {

            stereoEnabled = true

        } else {

            connectionMessage =
                "Nie udało się uruchomić stereo"
        }
    }


    // =====================================================
    // WYŁĄCZ TYLKO STEREO
    // =====================================================

    fun stopStereoPreview() {

        if (stereoEnabled) {

            rosBridgeClient
                .unsubscribeStereoCamera()
        }


        stereoEnabled = false

        stereoBitmap = null
    }


    // =====================================================
    // MAPA GŁĘBI ON / OFF
    // =====================================================

    fun toggleDepthPreview() {

        if (!isConnected) {

            connectionMessage =
                "Brak połączenia z robotem"

            return
        }


        // Jeśli już działa -> wyłącz
        if (depthEnabled) {

            rosBridgeClient
                .unsubscribeDepthCamera()

            depthEnabled = false

            depthBitmap = null

            return
        }


        // Jeśli działa stereo,
        // wyłączamy je przed mapą głębi.
        if (stereoEnabled) {

            rosBridgeClient
                .unsubscribeStereoCamera()

            stereoEnabled = false

            stereoBitmap = null
        }


        if (aiEnabled) {

            rosBridgeClient
                .unsubscribeAiCamera()

            aiEnabled = false
            aiBitmap = null
        }


        val success =
            rosBridgeClient
                .subscribeDepthCamera()


        if (success) {

            depthEnabled = true

        } else {

            connectionMessage =
                "Nie udało się uruchomić mapy głębi"
        }
    }


    // =====================================================
    // WYŁĄCZ TYLKO MAPĘ GŁĘBI
    // =====================================================

    fun stopDepthPreview() {

        if (depthEnabled) {

            rosBridgeClient
                .unsubscribeDepthCamera()
        }


        depthEnabled = false

        depthBitmap = null
    }


    // =====================================================
    // AI WYKRYWANIE KOSTKI ON / OFF
    // =====================================================

    fun toggleAiPreview() {

        if (!isConnected) {

            connectionMessage =
                "Brak połączenia z robotem"

            return
        }


        if (aiEnabled) {

            rosBridgeClient
                .unsubscribeAiCamera()

            aiEnabled = false
            aiBitmap = null

            return
        }


        if (stereoEnabled) {

            rosBridgeClient
                .unsubscribeStereoCamera()

            stereoEnabled = false
            stereoBitmap = null
        }


        if (depthEnabled) {

            rosBridgeClient
                .unsubscribeDepthCamera()

            depthEnabled = false
            depthBitmap = null
        }


        val success =
            rosBridgeClient
                .subscribeAiCamera()


        if (success) {

            aiEnabled = true

        } else {

            connectionMessage =
                "Nie udało się uruchomić podglądu AI"
        }
    }


    fun stopAiPreview() {

        if (aiEnabled) {

            rosBridgeClient
                .unsubscribeAiCamera()
        }

        aiEnabled = false
        aiBitmap = null

        arucoCubeX = null
        arucoCubeY = null
        arucoCubeDistanceM = null
        arucoBearingDeg = null
        arucoCubeYawDeg = null
        arucoQuality = null
        arucoCubeId = null
        arucoDetected = false
    }


    // =====================================================
    // WYŁĄCZ WSZYSTKIE PODGLĄDY AUTONOMICZNE
    // =====================================================

    fun stopAutonomousPreview() {

        if (stereoEnabled) {

            rosBridgeClient
                .unsubscribeStereoCamera()
        }


        if (depthEnabled) {

            rosBridgeClient
                .unsubscribeDepthCamera()
        }


        if (aiEnabled) {

            rosBridgeClient
                .unsubscribeAiCamera()
        }


        stereoEnabled = false

        stereoBitmap = null


        depthEnabled = false

        depthBitmap = null


        aiEnabled = false

        aiBitmap = null
    }


    // =====================================================
    // WIZJA DLA JAZDY AUTONOMICZNEJ
    //
    // W tym trybie DEPTH i AI działają RÓWNOCZEŚNIE.
    // Nie korzystamy tutaj z toggleDepthPreview()/toggleAiPreview(),
    // bo tamte funkcje celowo wyłączają drugi podgląd.
    // =====================================================

    fun startAutonomousDriveVision() {

        if (!isConnected) {

            connectionMessage =
                "Brak polaczenia z robotem"

            return
        }


        var depthOk = true
        var aiOk = true
        var aiPositionOk = true


        if (!depthEnabled) {

            depthOk =
                rosBridgeClient
                    .subscribeDepthCamera()

            if (depthOk) {
                depthEnabled = true
            }
        }


        if (!aiEnabled) {

            aiOk =
                rosBridgeClient
                    .subscribeAiCamera()

            if (aiOk) {
                aiEnabled = true
            }
        }


        // Osobna, lekka subskrypcja pozycji 3D kostki.
        // Jest niezalezna od obrazu debug AI.
        aiPositionOk =
            rosBridgeClient
                .subscribeAiCubePosition()


        connectionMessage =
            when {
                depthOk && aiOk && aiPositionOk ->
                    "Depth + AI + odleglosc uruchomione"

                else ->
                    "Wizja uruchomiona czesciowo"
            }
    }


    fun stopAutonomousDriveVision() {

        if (depthEnabled) {

            rosBridgeClient
                .unsubscribeDepthCamera()
        }


        if (aiEnabled) {

            rosBridgeClient
                .unsubscribeAiCamera()
        }


        rosBridgeClient
            .unsubscribeAiCubePosition()


        depthEnabled = false
        depthBitmap = null

        aiEnabled = false
        aiBitmap = null
    }



    // =====================================================
    // WIZJA EKRANU "SPRZĄTANIE"
    //
    // NIE używa /ai_cube/*.
    // Subskrybuje dokładnie ArUco + statusy cube_pick/cube_place/trunk.
    // =====================================================

    // =====================================================
    // OSOBNY PODGLĄD ARUCO W "SYSTEM WIZYJNY"
    // =====================================================

    fun startArucoPreview() {

        val debugOk =
            rosBridgeClient
                .subscribeArucoDebugCamera()

        val positionOk =
            rosBridgeClient
                .subscribeArucoPosition()

        val detectedOk =
            rosBridgeClient
                .subscribeArucoDetected()

        val idOk =
            rosBridgeClient
                .subscribeArucoId()

        val yawOk =
            rosBridgeClient
                .subscribeArucoYaw()

        val qualityOk =
            rosBridgeClient
                .subscribeArucoQuality()


        arucoEnabled =
            debugOk
                    &&
                    positionOk
                    &&
                    detectedOk
                    &&
                    idOk
                    &&
                    yawOk
                    &&
                    qualityOk
    }


    fun stopArucoPreview() {

        rosBridgeClient
            .unsubscribeArucoDebugCamera()

        rosBridgeClient
            .unsubscribeArucoPosition()

        rosBridgeClient
            .unsubscribeArucoDetected()

        rosBridgeClient
            .unsubscribeArucoId()

        rosBridgeClient
            .unsubscribeArucoYaw()

        rosBridgeClient
            .unsubscribeArucoQuality()


        arucoEnabled = false
        arucoBitmap = null

        arucoDetected = false
        arucoCubeId = null

        arucoCubeX = null
        arucoCubeY = null
        arucoCubeDistanceM = null

        arucoBearingDeg = null
        arucoCubeYawDeg = null
        arucoQuality = null
    }


    fun startCleanupVision() {

        if (!isConnected) {

            connectionMessage =
                "Brak polaczenia z robotem"

            return
        }

        var depthOk = true

        if (!depthEnabled) {

            depthOk =
                rosBridgeClient
                    .subscribeDepthCamera()

            if (depthOk) {
                depthEnabled = true
            }
        }

        val aiDebugOk =
            rosBridgeClient
                .subscribeAiCamera()

        if (aiDebugOk) {
            aiEnabled = true
        }

        val aiPositionOk =
            rosBridgeClient
                .subscribeAiFastPosition()

        val aiDetectedOk =
            rosBridgeClient
                .subscribeAiDetected()


        val arucoDebugOk =
            rosBridgeClient
                .subscribeArucoDebugCamera()

        if (arucoDebugOk) {
            arucoEnabled = true
        }

        val positionOk =
            rosBridgeClient
                .subscribeArucoPosition()

        val detectedOk =
            rosBridgeClient
                .subscribeArucoDetected()

        val idOk =
            rosBridgeClient
                .subscribeArucoId()

        val yawOk =
            rosBridgeClient
                .subscribeArucoYaw()

        val qualityOk =
            rosBridgeClient
                .subscribeArucoQuality()

        val pickStatusOk =
            rosBridgeClient
                .subscribeCubePickStatus()

        val placeStatusOk =
            rosBridgeClient
                .subscribeCubePlaceStatus()

        val trunkStatusOk =
            rosBridgeClient
                .subscribeTrunkLoadingStatus()

        connectionMessage =
            if (
                depthOk
                &&
                aiDebugOk
                &&
                aiPositionOk
                &&
                aiDetectedOk
                &&
                arucoDebugOk
                &&
                positionOk
                &&
                detectedOk
                &&
                idOk
                &&
                yawOk
                &&
                qualityOk
                &&
                pickStatusOk
                &&
                placeStatusOk
                &&
                trunkStatusOk
            ) {

                "Depth + AI jazda + ArUco pick gotowe"

            } else {

                "Cleanup vision uruchomiona czesciowo"
            }
    }


    fun stopCleanupVision() {

        rosBridgeClient
            .unsubscribeAiCamera()

        rosBridgeClient
            .unsubscribeAiFastPosition()

        rosBridgeClient
            .unsubscribeAiDetected()

        rosBridgeClient
            .unsubscribeArucoDebugCamera()

        rosBridgeClient
            .unsubscribeArucoPosition()

        rosBridgeClient
            .unsubscribeArucoDetected()

        rosBridgeClient
            .unsubscribeArucoId()

        rosBridgeClient
            .unsubscribeArucoYaw()

        rosBridgeClient
            .unsubscribeArucoQuality()

        rosBridgeClient
            .unsubscribeCubePickStatus()

        rosBridgeClient
            .unsubscribeCubePlaceStatus()

        rosBridgeClient
            .unsubscribeTrunkLoadingStatus()

        if (depthEnabled) {

            rosBridgeClient
                .unsubscribeDepthCamera()
        }

        depthEnabled = false
        depthBitmap = null

        aiEnabled = false
        aiBitmap = null

        aiDriveDetected = false
        aiDriveX = null
        aiDriveY = null
        aiDriveDistanceM = null
        aiDriveBearingDeg = null

        arucoEnabled = false
        arucoBitmap = null
    }


    // =====================================================
    // JAZDA PÓŁAUTONOMICZNA
    //
    // Telefon daje kierunek jak w manualu, ale komendy jadą przez
    // /autonomous_drive/command. Raspberry Pi pilnuje 10 cm:
    // HC-SR04 + /stereo/center_distance.
    // =====================================================

    fun startSemiAutonomousMode() {

        if (!isConnected) {
            connectionMessage = "Brak połączenia z robotem"
            return
        }

        // Stara misja autonomiczna nie może równocześnie sterować platformą.
        rosBridgeClient
            .publishCleanupMissionCommand(
                "stop"
            )

        // Kamera ekranu półautonomicznego = tylko AI detection cube.
        val aiCameraOk =
            rosBridgeClient
                .subscribeAiCamera()

        if (aiCameraOk) {
            aiEnabled = true
        }

        // Dane do panelu.
        val depthDistanceOk =
            rosBridgeClient
                .subscribeDepthCenterDistance()

        val aiPositionOk =
            rosBridgeClient
                .subscribeAiFastPosition()

        val aiDetectedOk =
            rosBridgeClient
                .subscribeAiDetected()

        val arucoDebugOk =
            rosBridgeClient
                .subscribeArucoDebugCamera()

        val arucoPositionOk =
            rosBridgeClient
                .subscribeArucoPosition()

        val arucoDetectedOk =
            rosBridgeClient
                .subscribeArucoDetected()

        val arucoIdOk =
            rosBridgeClient
                .subscribeArucoId()

        val arucoYawOk =
            rosBridgeClient
                .subscribeArucoYaw()

        val arucoQualityOk =
            rosBridgeClient
                .subscribeArucoQuality()

        rosBridgeClient
            .subscribeCubePickStatus()

        // Najpierw subskrypcja odczytów HC-SR04.
        // Sam node półautonomiczny wysyła @U:1 po komendzie "start".
        val ultrasonicOk =
            if (!ultrasonicEnabled) {
                rosBridgeClient
                    .subscribeUltrasonic()
            } else {
                true
            }

        if (ultrasonicOk) {
            ultrasonicEnabled = true
        }

        arucoEnabled =
            arucoDebugOk
                    &&
                    arucoPositionOk
                    &&
                    arucoDetectedOk
                    &&
                    arucoIdOk
                    &&
                    arucoYawOk
                    &&
                    arucoQualityOk

        val started =
            rosBridgeClient
                .publishAutonomousDriveCommand(
                    "start"
                )

        connectionMessage =
            if (
                started
                &&
                aiCameraOk
                &&
                depthDistanceOk
                &&
                aiPositionOk
                &&
                aiDetectedOk
                &&
                ultrasonicOk
            ) {
                "Jazda półautonomiczna gotowa"
            } else {
                "Jazda półautonomiczna uruchomiona częściowo"
            }
    }


    fun stopSemiAutonomousMode() {

        if (isConnected) {
            rosBridgeClient
                .publishAutonomousDriveCommand(
                    "stop"
                )
        }

        if (aiEnabled) {
            rosBridgeClient
                .unsubscribeAiCamera()
        }

        aiEnabled = false
        aiBitmap = null

        rosBridgeClient
            .unsubscribeDepthCenterDistance()

        rosBridgeClient
            .unsubscribeAiFastPosition()

        rosBridgeClient
            .unsubscribeAiDetected()

        rosBridgeClient
            .unsubscribeArucoDebugCamera()

        rosBridgeClient
            .unsubscribeArucoPosition()

        rosBridgeClient
            .unsubscribeArucoDetected()

        rosBridgeClient
            .unsubscribeArucoId()

        rosBridgeClient
            .unsubscribeArucoYaw()

        rosBridgeClient
            .unsubscribeArucoQuality()

        rosBridgeClient
            .unsubscribeCubePickStatus()

        if (ultrasonicEnabled) {
            rosBridgeClient
                .unsubscribeUltrasonic()
        }

        ultrasonicEnabled = false

        depthCenterDistanceM = null

        ultrasonicFrontLeft = null
        ultrasonicFrontCenter = null
        ultrasonicFrontRight = null
        ultrasonicLeft = null
        ultrasonicRight = null
        ultrasonicRear = null

        aiDriveDetected = false
        aiDriveX = null
        aiDriveY = null
        aiDriveDistanceM = null
        aiDriveBearingDeg = null

        arucoEnabled = false
        arucoBitmap = null
        arucoDetected = false
        arucoCubeId = null
        arucoCubeX = null
        arucoCubeY = null
        arucoCubeDistanceM = null
        arucoBearingDeg = null
        arucoCubeYawDeg = null
        arucoQuality = null
    }


    fun sendSemiAutonomousCommand(
        command: String
    ) {

        if (!isConnected) {
            connectionMessage = "Brak połączenia z robotem"
            return
        }

        val sent =
            rosBridgeClient
                .publishAutonomousDriveCommand(
                    command
                )

        if (!sent) {
            connectionMessage =
                "Nie udało się wysłać komendy półautonomicznej"
        }
    }


    fun startSemiAutonomousPick() {

        if (!isConnected) {
            connectionMessage = "Brak połączenia z robotem"
            return
        }

        val sent =
            rosBridgeClient
                .publishAutonomousDriveCommand(
                    "pick"
                )

        connectionMessage =
            if (sent) {
                "Podnoszenie kostki uruchomione"
            } else {
                "Nie udało się uruchomić podnoszenia"
            }
    }


    // =====================================================
    // WYSYŁANIE KOMEND
    // =====================================================

    fun sendManipulatorCommand(
        command: String
    ) {

        if (!isConnected) {

            connectionMessage =
                "Brak połączenia z robotem"

            return
        }


        val sent =
            rosBridgeClient
                .publishManipulatorCommand(
                    command
                )


        if (!sent) {

            connectionMessage =
                "Nie udało się wysłać komendy"
        }
    }


    // =====================================================
    // EKRAN "PODNIESIENIE KOSTKI"
    // =====================================================

    fun startCubePickScreen() {

        if (!isConnected) {
            connectionMessage = "Brak połączenia z robotem"
            return
        }

        startArucoPreview()

        rosBridgeClient.subscribeCubePickStatus()
        rosBridgeClient.subscribeCubePlaceStatus()
    }


    fun stopCubePickScreen() {

        stopArucoPreview()

        rosBridgeClient.unsubscribeCubePickStatus()
        rosBridgeClient.unsubscribeCubePlaceStatus()
    }


    fun startCubePickFull() {

        if (!isConnected) {
            connectionMessage = "Brak połączenia z robotem"
            return
        }

        val sent =
            rosBridgeClient.publishCubePickCommand(
                "full"
            )

        connectionMessage =
            if (sent) {
                "Podnoszenie kostki uruchomione"
            } else {
                "Nie udało się uruchomić cube_pick"
            }
    }


    fun sendCubeToTrunk() {

        if (!isConnected) {
            connectionMessage = "Brak połączenia z robotem"
            return
        }

        // Kostka jest już podniesiona przez cube_pick.
        // Dlatego NIE uruchamiamy trunk_loading (bo zrobiłby pick drugi raz),
        // tylko bezpośrednio cube_place -> bagażnik.
        val sent =
            rosBridgeClient.publishCubePlaceCommand(
                "start"
            )

        connectionMessage =
            if (sent) {
                "Przenoszenie kostki do bagażnika uruchomione"
            } else {
                "Nie udało się uruchomić cube_place"
            }
    }


    fun startTrunkLoading() {

        if (!isConnected) {

            connectionMessage =
                "Brak polaczenia z robotem"

            return
        }


        val sent =
            rosBridgeClient
                .publishTrunkLoadingCommand(
                    "start"
                )


        connectionMessage =
            if (sent) {

                "Pelny zaladunek do bagaznika uruchomiony"

            } else {

                "Nie udalo sie uruchomic zaladunku"
            }
    }


    // =====================================================
    // HC-SR04 ON / OFF
    //
    // To steruje subskrypcją danych w aplikacji.
    // Odczyty mogą działać niezależnie od jazdy autonomicznej.
    // =====================================================

    fun toggleUltrasonicSensors() {

        if (!isConnected) {

            connectionMessage =
                "Brak polaczenia z robotem"

            return
        }


        // =====================================================
        // WYŁĄCZANIE HC-SR04
        // =====================================================

        if (ultrasonicEnabled) {

            // Jezeli misja zbierania dziala, najpierw ja zatrzymujemy.
            stopCleanupMission()

            // Zachowujemy tez STOP starego autonomous_drive dla zgodnosci.
            stopAutonomousDrive()

            // Fizycznie wyłącz pomiary HC-SR04 w Arduino.
            //
            // serial_trajectory_bridge przepuszcza komendy zaczynające
            // się od @ bezpośrednio do Arduino.
            val commandSent =
                rosBridgeClient
                    .publishManipulatorCommand(
                        "@U:0"
                    )


            // Przestań odbierać /robot/ultrasonic w aplikacji.
            rosBridgeClient
                .unsubscribeUltrasonic()


            ultrasonicEnabled = false

            ultrasonicFrontLeft = null
            ultrasonicFrontCenter = null
            ultrasonicFrontRight = null
            ultrasonicLeft = null
            ultrasonicRight = null
            ultrasonicRear = null


            connectionMessage =
                if (commandSent) {

                    "Czujniki HC-SR04 wylaczone"

                } else {

                    "HC-SR04: blad wyslania @U:0"
                }

            return
        }


        // =====================================================
        // WŁĄCZANIE HC-SR04
        //
        // WAŻNA KOLEJNOŚĆ:
        // 1. najpierw subskrypcja ROS,
        // 2. potem @U:1 do Arduino.
        //
        // Dzięki temu nie zgubimy pierwszej ramki pomiarowej.
        // =====================================================

        val subscribed =
            rosBridgeClient
                .subscribeUltrasonic()


        if (!subscribed) {

            connectionMessage =
                "Nie udalo sie zasubskrybowac HC-SR04"

            return
        }


        val commandSent =
            rosBridgeClient
                .publishManipulatorCommand(
                    "@U:1"
                )


        if (!commandSent) {

            rosBridgeClient
                .unsubscribeUltrasonic()

            connectionMessage =
                "Nie udalo sie wyslac @U:1 do Arduino"

            return
        }


        ultrasonicEnabled = true

        connectionMessage =
            "Czujniki HC-SR04 wlaczone"
    }


    fun stopUltrasonicSensors() {

        if (ultrasonicEnabled) {

            rosBridgeClient
                .unsubscribeUltrasonic()
        }

        ultrasonicEnabled = false

        ultrasonicFrontLeft = null
        ultrasonicFrontCenter = null
        ultrasonicFrontRight = null
        ultrasonicLeft = null
        ultrasonicRight = null
        ultrasonicRear = null
    }


    // =====================================================
    // AUTONOMICZNE ZBIERANIE - START / STOP
    // =====================================================

    fun startCleanupMission() {

        if (!isConnected) {

            connectionMessage =
                "Brak polaczenia z robotem"

            return
        }


        // =====================================================
        // HC-SR04 WŁĄCZAJĄ SIĘ AUTOMATYCZNIE RAZEM Z MISJĄ.
        // =====================================================
        //
        // Kolejność:
        // 1. subskrypcja /robot/ultrasonic,
        // 2. @U:1 do Arduino,
        // 3. START cleanup_mission.
        // =====================================================

        if (!ultrasonicEnabled) {

            val subscribed =
                rosBridgeClient
                    .subscribeUltrasonic()

            if (!subscribed) {

                connectionMessage =
                    "Nie udalo sie zasubskrybowac HC-SR04"

                return
            }


            val ultrasonicCommandSent =
                rosBridgeClient
                    .publishManipulatorCommand(
                        "@U:1"
                    )

            if (!ultrasonicCommandSent) {

                rosBridgeClient
                    .unsubscribeUltrasonic()

                connectionMessage =
                    "Nie udalo sie wlaczyc HC-SR04"

                return
            }

            ultrasonicEnabled = true
        }


        val sent =
            rosBridgeClient
                .publishCleanupMissionCommand(
                    "start"
                )


        connectionMessage =
            if (sent) {

                "Autonomiczne zbieranie uruchomione"

            } else {

                "Nie udalo sie uruchomic zbierania"
            }
    }


    fun stopCleanupMission() {

        if (!isConnected) {
            return
        }


        val sent =
            rosBridgeClient
                .publishCleanupMissionCommand(
                    "stop"
                )


        // HC-SR04 wyłączamy razem z zatrzymaniem misji.
        if (ultrasonicEnabled) {

            rosBridgeClient
                .publishManipulatorCommand(
                    "@U:0"
                )

            rosBridgeClient
                .unsubscribeUltrasonic()

            ultrasonicEnabled = false

            ultrasonicFrontLeft = null
            ultrasonicFrontCenter = null
            ultrasonicFrontRight = null
            ultrasonicLeft = null
            ultrasonicRight = null
            ultrasonicRear = null
        }


        connectionMessage =
            if (sent) {

                "Autonomiczne zbieranie zatrzymane"

            } else {

                "Nie udalo sie zatrzymac zbierania"
            }
    }


    fun resetCleanupCounter() {

        if (!isConnected) {
            return
        }

        rosBridgeClient
            .publishCleanupMissionCommand(
                "reset_count"
            )
    }


    fun startAutonomousDrive() {

        if (!isConnected) {

            connectionMessage =
                "Brak polaczenia z robotem"

            return
        }


        // Jazdy autonomicznej NIE wolno uruchomic,
        // dopóki użytkownik nie włączy HC-SR04.
        if (!ultrasonicEnabled) {

            connectionMessage =
                "Najpierw wlacz czujniki HC-SR04"

            autonomousDriveStatus =
                "WŁĄCZ HC-SR04"

            return
        }


        val sent =
            rosBridgeClient
                .publishAutonomousDriveCommand(
                    "start"
                )


        connectionMessage =
            if (sent) {

                "Jazda autonomiczna uruchomiona"

            } else {

                "Nie udalo sie uruchomic jazdy autonomicznej"
            }
    }


    fun stopAutonomousDrive() {

        if (!isConnected) {

            connectionMessage =
                "Brak polaczenia z robotem"

            return
        }


        val sent =
            rosBridgeClient
                .publishAutonomousDriveCommand(
                    "stop"
                )


        connectionMessage =
            if (sent) {

                "Jazda autonomiczna zatrzymana"

            } else {

                "Nie udalo sie zatrzymac jazdy autonomicznej"
            }
    }


    fun sendTestMessage() {

        if (!isConnected) {

            connectionMessage =
                "Najpierw połącz się z robotem"

            return
        }


        val sent =
            rosBridgeClient
                .publishTestMessage()


        connectionMessage =
            if (sent) {

                "Wysłano wiadomość testową"

            } else {

                "Nie udało się wysłać wiadomości"
            }
    }


    fun toggleUpperFan() {

        sendManipulatorCommand(
            "5"
        )
    }


    fun toggleLowerFan() {

        sendManipulatorCommand(
            "6"
        )
    }


    fun setBuzzer(
        enabled: Boolean
    ) {

        sendManipulatorCommand(
            if (enabled) {
                "7"
            } else {
                "8"
            }
        )
    }


    // =====================================================
    // CALLBACKI ROSBRIDGE
    // =====================================================

    override fun onConnected() {

        mainHandler.post {

            connectionState =
                ConnectionState.CONNECTED


            connectionMessage =
                "Połączony z robotem"
        }
    }


    override fun onDisconnected() {

        mainHandler.post {

            connectionState =
                ConnectionState.DISCONNECTED


            connectionMessage =
                "Rozłączony"


            cameraBitmap = null

            jointPositionsRad =
                List(
                    5
                ) {
                    null
                }

            tcpXmm = null
            tcpYmm = null
            tcpZmm = null


            stereoBitmap = null
            stereoEnabled = false


            depthBitmap = null
            depthEnabled = false

            aiBitmap = null
            aiEnabled = false


            upperTemperature = null
            lowerTemperature = null

            upperHumidity = null
            lowerHumidity = null


            upperFanEnabled = false
            lowerFanEnabled = false

            buzzerEnabled = false

            ultrasonicEnabled = false
            ultrasonicFrontLeft = null
            ultrasonicFrontCenter = null
            ultrasonicFrontRight = null
            ultrasonicLeft = null
            ultrasonicRight = null
            ultrasonicRear = null

            cleanupMissionStatus = "STOP"
            cubesLoaded = 0

            arucoBitmap = null
            arucoEnabled = false

            arucoDetected = false
            arucoCubeId = null
            arucoCubeX = null
            arucoCubeY = null
            arucoCubeDistanceM = null
            arucoBearingDeg = null
            arucoCubeYawDeg = null
            arucoQuality = null

            cubePickStatus = "idle"
            cubePlaceStatus = "idle"
            trunkLoadingStatus = "idle"
        }
    }


    override fun onError(
        message: String
    ) {

        mainHandler.post {

            connectionState =
                ConnectionState.ERROR


            connectionMessage =
                "Błąd: $message"


            cameraBitmap = null


            stereoBitmap = null
            stereoEnabled = false


            depthBitmap = null
            depthEnabled = false

            aiBitmap = null
            aiEnabled = false

            arucoBitmap = null
            arucoEnabled = false
        }
    }


    // =====================================================
    // WIADOMOŚCI ROS
    // =====================================================

    override fun onMessage(
        message: String
    ) {

        try {

            val json =
                JSONObject(
                    message
                )


            val operation =
                json.optString(
                    "op"
                )


            val topic =
                json.optString(
                    "topic"
                )


            // =================================================
            // MANUAL - AKTUALNE JOINTY / TCP
            // =================================================

            if (
                operation == "publish"
                &&
                topic == "/joint_states"
            ) {

                decodeJointStates(
                    json
                )

                return
            }


            // =================================================
            // HC-SR04
            // =================================================

            if (
                operation == "publish"
                &&
                topic == "/robot/ultrasonic"
            ) {

                decodeUltrasonic(
                    json
                )

                return
            }


            // =================================================
            // MAPA GŁĘBI - ODLEGŁOŚĆ CENTRALNA
            // =================================================

            if (
                operation == "publish"
                &&
                topic == "/stereo/center_distance"
            ) {

                decodeDepthCenterDistance(
                    json
                )

                return
            }


            // =================================================
            // STATUS AUTONOMICZNEGO ZBIERANIA
            // =================================================

            if (
                operation == "publish"
                &&
                topic == "/cleanup_mission/status"
            ) {

                decodeCleanupMissionStatus(
                    json
                )

                return
            }


            // =================================================
            // LICZNIK KOSTEK W BAGAZNIKU
            // =================================================

            if (
                operation == "publish"
                &&
                topic == "/cleanup_mission/cubes_loaded"
            ) {

                decodeCleanupCubeCount(
                    json
                )

                return
            }


            // =================================================
            // CLEANUP - DANE ARUCO UŻYWANE PRZEZ CUBE_PICK
            // =================================================

            if (
                operation == "publish"
                &&
                topic == "/cube_aruco/position_camera"
            ) {
                decodeArucoPosition(json)
                return
            }

            if (
                operation == "publish"
                &&
                topic == "/cube_aruco/detected"
            ) {
                decodeArucoDetected(json)
                return
            }

            if (
                operation == "publish"
                &&
                topic == "/cube_aruco/id"
            ) {
                decodeArucoId(json)
                return
            }

            if (
                operation == "publish"
                &&
                topic == "/cube_aruco/yaw_deg"
            ) {
                decodeArucoYaw(json)
                return
            }

            if (
                operation == "publish"
                &&
                topic == "/cube_aruco/quality"
            ) {
                decodeArucoQuality(json)
                return
            }

            if (
                operation == "publish"
                &&
                topic == "/cube_pick/status"
            ) {
                decodeSimpleStringStatus(
                    json
                ) {
                    cubePickStatus = it
                }
                return
            }

            if (
                operation == "publish"
                &&
                topic == "/cube_place/status"
            ) {
                decodeSimpleStringStatus(
                    json
                ) {
                    cubePlaceStatus = it
                }
                return
            }

            if (
                operation == "publish"
                &&
                topic == "/trunk_loading/status"
            ) {
                decodeSimpleStringStatus(
                    json
                ) {
                    trunkLoadingStatus = it
                }
                return
            }


            // =================================================
            // STATUS JAZDY AUTONOMICZNEJ
            // =================================================

            if (
                operation == "publish"
                &&
                topic == "/autonomous_drive/status"
            ) {

                decodeAutonomousDriveStatus(
                    json
                )

                return
            }


            // =================================================
            // STATUS ROBOTA
            // =================================================

            if (
                operation == "publish"
                &&
                topic == "/robot/status"
            ) {

                decodeRobotStatus(
                    json
                )

                return
            }


            // =================================================
            // PRAWA KAMERA
            // =================================================

            if (
                operation == "publish"
                &&
                topic ==
                "/camera_right/right_camera/image_raw/compressed"
            ) {

                decodeRightCameraFrame(
                    json
                )

                return
            }


            // =================================================
            // STEREO
            // =================================================

            if (
                operation == "publish"
                &&
                topic ==
                "/stereo/rectified_check/compressed"
            ) {

                decodeStereoCameraFrame(
                    json
                )

                return
            }


            // =================================================
            // MAPA GŁĘBI
            // =================================================

            if (
                operation == "publish"
                &&
                topic ==
                "/stereo/depth/compressed"
            ) {

                decodeDepthCameraFrame(
                    json
                )

                return
            }


            // =================================================
            // AI - DOKLADNA POZYCJA KOSTKI / ODLEGLOSC Z
            // =================================================

            if (
                operation == "publish"
                &&
                topic == "/ai_cube/position_camera"
            ) {

                decodeAiCubePosition(
                    json
                )

                return
            }


            if (
                operation == "publish"
                &&
                topic == "/ai_cube/position_camera_fast"
            ) {

                decodeAiDrivePosition(json)
                return
            }


            if (
                operation == "publish"
                &&
                topic == "/ai_cube/detected"
            ) {

                decodeAiDriveDetected(json)
                return
            }


            // =================================================
            // ARUCO DEBUG - TEN SAM DETEKTOR CO CUBE_PICK
            // =================================================

            if (
                operation == "publish"
                &&
                topic ==
                "/cube_aruco/debug/compressed"
            ) {

                decodeArucoCameraFrame(
                    json
                )

                return
            }


            // =================================================
            // AI - WYKRYWANIE KOSTKI
            // =================================================

            if (
                operation == "publish"
                &&
                topic ==
                "/ai_cube/debug/compressed"
            ) {

                decodeAiCameraFrame(
                    json
                )

                return
            }


            mainHandler.post {

                lastRosMessage =
                    message
            }


        } catch (
            exception: Exception
        ) {

            println(
                "Błąd odbierania wiadomości ROS: " +
                        exception.message
            )
        }
    }


    // =====================================================
    // /joint_states -> KĄTY + AKTUALNY TCP
    // =====================================================

    private fun decodeJointStates(
        json: JSONObject
    ) {

        try {

            val rosMessage =
                json.optJSONObject(
                    "msg"
                )
                    ?: return


            val names =
                rosMessage.optJSONArray(
                    "name"
                )
                    ?: return


            val positions =
                rosMessage.optJSONArray(
                    "position"
                )
                    ?: return


            val values =
                mutableMapOf<
                        String,
                        Double
                        >()


            val count =
                minOf(
                    names.length(),
                    positions.length()
                )


            for (
            i
            in
            0
                    until
                    count
            ) {

                val name =
                    names.optString(
                        i
                    )


                if (
                    name.isBlank()
                ) {

                    continue
                }


                val value =
                    positions.optDouble(
                        i,
                        Double.NaN
                    )


                if (
                    value.isFinite()
                ) {

                    values[
                        name
                    ] =
                        value
                }
            }


            val q =
                listOf(
                    values[
                        "joint_1"
                    ],
                    values[
                        "joint_2"
                    ],
                    values[
                        "joint_3"
                    ],
                    values[
                        "joint_4"
                    ],
                    values[
                        "joint_5"
                    ]
                )


            mainHandler.post {

                jointPositionsRad =
                    q


                if (
                    q.all {
                        it != null
                    }
                ) {

                    updateTcpFromJointStates(
                        q.map {
                            it!!
                        }
                    )
                }
            }

        } catch (
            exception: Exception
        ) {

            println(
                "Błąd /joint_states: " +
                        exception.message
            )
        }
    }


    private fun updateTcpFromJointStates(
        q: List<Double>
    ) {

        if (q.size < 5) {
            return
        }

        val q1 = q[0]
        val q2 = q[1]
        val q3 = q[2]
        val q4 = q[3]

        val a2 = q2 - j2StraightRad
        val a23 = a2 + (q3 - j3StraightRad)
        val a234 = a23 + (q4 - j4StraightRad)

        val c1 = cos(q1)
        val s1 = sin(q1)
        val finalLength = j4J5M + j5TcpM

        // Dokładnie ten sam FK pozycji TCP co w robot_motion/arm_kinematics.py.
        val x =
            s1 * j2J3M * sin(a2) +
                    c1 * j3J4SideM +
                    s1 * j3J4M * sin(a23) +
                    s1 * finalLength * sin(a234)

        val y =
            -c1 * j2J3M * sin(a2) +
                    s1 * j3J4SideM -
                    c1 * j3J4M * sin(a23) -
                    c1 * finalLength * sin(a234)

        val z =
            j1J2M +
                    j2J3M * cos(a2) +
                    j3J4M * cos(a23) +
                    finalLength * cos(a234)

        tcpXmm = x * 1000.0
        tcpYmm = y * 1000.0
        tcpZmm = z * 1000.0
    }


    // =====================================================
    // HC-SR04
    // =====================================================

    private fun decodeDepthCenterDistance(
        json: JSONObject
    ) {

        try {

            val rosMessage =
                json.optJSONObject(
                    "msg"
                )
                    ?: return

            val value =
                rosMessage.optDouble(
                    "data",
                    Double.NaN
                )

            mainHandler.post {

                depthCenterDistanceM =
                    if (
                        value.isFinite()
                        &&
                        value > 0.0
                        &&
                        value < 10.0
                    ) {
                        value.toFloat()
                    } else {
                        null
                    }
            }

        } catch (
            exception: Exception
        ) {

            println(
                "Błąd /stereo/center_distance: " +
                        exception.message
            )
        }
    }


    private fun decodeUltrasonic(
        json: JSONObject
    ) {

        try {

            val rosMessage =
                json.optJSONObject(
                    "msg"
                )
                    ?: return


            val dataText =
                rosMessage.optString(
                    "data"
                )


            if (
                dataText.isBlank()
            ) {

                return
            }


            val data =
                JSONObject(
                    dataText
                )


            if (
                !data.optBoolean(
                    "ultrasonic",
                    false
                )
            ) {

                return
            }


            val frontLeft =
                readNullableFloat(
                    data,
                    "frontLeft"
                )

            val frontCenter =
                readNullableFloat(
                    data,
                    "frontCenter"
                )

            val frontRight =
                readNullableFloat(
                    data,
                    "frontRight"
                )

            val left =
                readNullableFloat(
                    data,
                    "left"
                )

            val right =
                readNullableFloat(
                    data,
                    "right"
                )

            val rear =
                readNullableFloat(
                    data,
                    "rear"
                )


            mainHandler.post {

                ultrasonicFrontLeft =
                    frontLeft

                ultrasonicFrontCenter =
                    frontCenter

                ultrasonicFrontRight =
                    frontRight

                ultrasonicLeft =
                    left

                ultrasonicRight =
                    right

                ultrasonicRear =
                    rear
            }


        } catch (
            exception: Exception
        ) {

            println(
                "Błąd dekodowania HC-SR04: " +
                        exception.message
            )
        }
    }


    // =====================================================
    // STATUS JAZDY AUTONOMICZNEJ
    // =====================================================

    // =====================================================
    // /ai_cube/position_camera
    //
    // PointStamped:
    //   msg.point.x
    //   msg.point.y
    //   msg.point.z  <-- TA WARTOSC NAS INTERESUJE
    // =====================================================

    // =====================================================
    // ARUCO /cube_aruco/position_camera
    //
    // X/Y/Z są dokładnie tym samym PointStamped, który czyta cube_pick.
    // =====================================================

    private fun decodeArucoPosition(
        json: JSONObject
    ) {

        try {

            val rosMessage =
                json.optJSONObject(
                    "msg"
                )
                    ?: return

            val point =
                rosMessage.optJSONObject(
                    "point"
                )
                    ?: return

            val x =
                point.optDouble(
                    "x",
                    Double.NaN
                )

            val y =
                point.optDouble(
                    "y",
                    Double.NaN
                )

            val z =
                point.optDouble(
                    "z",
                    Double.NaN
                )

            if (
                !x.isFinite()
                ||
                !y.isFinite()
                ||
                !z.isFinite()
                ||
                z <= 0.0
                ||
                z > 5.0
            ) {
                return
            }

            // TYLKO interpretacja LEWO/PRAWO dla platformy.
            // Surowe ArUco X/Y/Z pozostają bez zmian.
            val bearingDeg =
                Math.toDegrees(
                    atan2(
                        -x,
                        z
                    )
                )

            mainHandler.post {

                arucoCubeX =
                    x.toFloat()

                arucoCubeY =
                    y.toFloat()

                arucoCubeDistanceM =
                    z.toFloat()

                arucoBearingDeg =
                    bearingDeg.toFloat()
            }

        } catch (
            exception: Exception
        ) {

            println(
                "Blad /cube_aruco/position_camera: " +
                        exception.message
            )
        }
    }


    private fun decodeArucoDetected(
        json: JSONObject
    ) {

        val rosMessage =
            json.optJSONObject(
                "msg"
            )
                ?: return

        val detected =
            rosMessage.optBoolean(
                "data",
                false
            )

        mainHandler.post {
            arucoDetected = detected
        }
    }


    private fun decodeArucoId(
        json: JSONObject
    ) {

        val rosMessage =
            json.optJSONObject(
                "msg"
            )
                ?: return

        val id =
            rosMessage.optInt(
                "data",
                -1
            )

        mainHandler.post {
            arucoCubeId =
                if (id >= 0) {
                    id
                } else {
                    null
                }
        }
    }


    private fun decodeArucoYaw(
        json: JSONObject
    ) {

        val rosMessage =
            json.optJSONObject(
                "msg"
            )
                ?: return

        val value =
            rosMessage.optDouble(
                "data",
                Double.NaN
            )

        if (!value.isFinite()) {
            return
        }

        mainHandler.post {
            arucoCubeYawDeg =
                value.toFloat()
        }
    }


    private fun decodeArucoQuality(
        json: JSONObject
    ) {

        val rosMessage =
            json.optJSONObject(
                "msg"
            )
                ?: return

        val value =
            rosMessage.optDouble(
                "data",
                Double.NaN
            )

        if (!value.isFinite()) {
            return
        }

        mainHandler.post {
            arucoQuality =
                value.toFloat()
        }
    }


    private fun decodeSimpleStringStatus(
        json: JSONObject,
        setter: (String) -> Unit
    ) {

        val rosMessage =
            json.optJSONObject(
                "msg"
            )
                ?: return

        val value =
            rosMessage.optString(
                "data"
            )

        if (value.isBlank()) {
            return
        }

        mainHandler.post {
            setter(value)
        }
    }


    // Stary AI screen może nadal subskrybować /ai_cube/position_camera.
    // Na ekranie cleanup NIE używamy tego topicu.
    private fun decodeAiDriveDetected(
        json: JSONObject
    ) {

        val rosMessage =
            json.optJSONObject("msg")
                ?: return

        val detected =
            rosMessage.optBoolean(
                "data",
                false
            )

        mainHandler.post {
            aiDriveDetected = detected

            if (!detected) {
                aiDriveBearingDeg = null
            }
        }
    }


    private fun decodeAiDrivePosition(
        json: JSONObject
    ) {

        val rosMessage =
            json.optJSONObject("msg")
                ?: return

        val point =
            rosMessage.optJSONObject("point")
                ?: return

        val x =
            point.optDouble(
                "x",
                Double.NaN
            )

        val y =
            point.optDouble(
                "y",
                Double.NaN
            )

        val z =
            point.optDouble(
                "z",
                Double.NaN
            )

        if (
            !x.isFinite()
            ||
            !y.isFinite()
            ||
            !z.isFinite()
            ||
            z <= 0.0
        ) {
            return
        }

        val bearingDeg =
            Math.toDegrees(
                atan2(
                    x,
                    z
                )
            )

        mainHandler.post {

            aiDriveX =
                x.toFloat()

            aiDriveY =
                y.toFloat()

            aiDriveDistanceM =
                z.toFloat()

            aiDriveBearingDeg =
                bearingDeg.toFloat()
        }
    }


    private fun decodeAiCubePosition(
        json: JSONObject
    ) {
        // Celowo bez zapisu do stanu cleanup.
    }


    private fun decodeCleanupMissionStatus(
        json: JSONObject
    ) {

        try {

            val rosMessage =
                json.optJSONObject(
                    "msg"
                )
                    ?: return

            val status =
                rosMessage.optString(
                    "data"
                )

            if (status.isBlank()) {
                return
            }

            mainHandler.post {
                cleanupMissionStatus = status
            }

        } catch (
            exception: Exception
        ) {

            println(
                "Blad statusu cleanup mission: " +
                        exception.message
            )
        }
    }


    private fun decodeCleanupCubeCount(
        json: JSONObject
    ) {

        try {

            val rosMessage =
                json.optJSONObject(
                    "msg"
                )
                    ?: return

            val count =
                rosMessage.optInt(
                    "data",
                    0
                )

            mainHandler.post {
                cubesLoaded = count
            }

        } catch (
            exception: Exception
        ) {

            println(
                "Blad licznika kostek: " +
                        exception.message
            )
        }
    }


    private fun decodeAutonomousDriveStatus(
        json: JSONObject
    ) {

        try {

            val rosMessage =
                json.optJSONObject(
                    "msg"
                )
                    ?: return


            val status =
                rosMessage.optString(
                    "data"
                )


            if (
                status.isBlank()
            ) {

                return
            }


            mainHandler.post {

                autonomousDriveStatus =
                    status
            }


        } catch (
            exception: Exception
        ) {

            println(
                "Błąd statusu jazdy autonomicznej: " +
                        exception.message
            )
        }
    }


    // =====================================================
    // STATUS ROBOTA
    // =====================================================

    private fun decodeRobotStatus(
        json: JSONObject
    ) {

        try {

            val rosMessage =
                json.optJSONObject(
                    "msg"
                )
                    ?: return


            val statusText =
                rosMessage.optString(
                    "data"
                )


            if (
                statusText.isBlank()
            ) {

                return
            }


            val status =
                JSONObject(
                    statusText
                )


            val newUpperTemperature =
                readNullableFloat(
                    status,
                    "upperTemp"
                )


            val newLowerTemperature =
                readNullableFloat(
                    status,
                    "lowerTemp"
                )


            val newUpperHumidity =
                readNullableFloat(
                    status,
                    "upperHumidity"
                )


            val newLowerHumidity =
                readNullableFloat(
                    status,
                    "lowerHumidity"
                )


            val newUpperFanEnabled =
                status.optBoolean(
                    "upperFan",
                    false
                )


            val newLowerFanEnabled =
                status.optBoolean(
                    "lowerFan",
                    false
                )


            val newBuzzerEnabled =
                status.optBoolean(
                    "buzzer",
                    false
                )


            mainHandler.post {

                upperTemperature =
                    newUpperTemperature


                lowerTemperature =
                    newLowerTemperature


                upperHumidity =
                    newUpperHumidity


                lowerHumidity =
                    newLowerHumidity


                upperFanEnabled =
                    newUpperFanEnabled


                lowerFanEnabled =
                    newLowerFanEnabled


                buzzerEnabled =
                    newBuzzerEnabled
            }


        } catch (
            exception: Exception
        ) {

            println(
                "Błąd dekodowania statusu robota: " +
                        exception.message
            )
        }
    }


    private fun readNullableFloat(
        json: JSONObject,
        key: String
    ): Float? {

        if (
            !json.has(key)
            ||
            json.isNull(key)
        ) {

            return null
        }


        val value =
            json.optDouble(
                key,
                Double.NaN
            )


        return if (
            value.isNaN()
        ) {

            null

        } else {

            value.toFloat()
        }
    }


    // =====================================================
    // PRAWA KAMERA
    // =====================================================

    private fun decodeRightCameraFrame(
        json: JSONObject
    ) {

        if (
            isDecodingCameraFrame
        ) {

            return
        }


        isDecodingCameraFrame =
            true


        try {

            val bitmap =
                decodeCompressedImage(
                    json
                )
                    ?: return


            mainHandler.post {

                cameraBitmap =
                    bitmap
            }


        } catch (
            exception: Exception
        ) {

            println(
                "Błąd dekodowania prawej kamery: " +
                        exception.message
            )


        } finally {

            isDecodingCameraFrame =
                false
        }
    }


    // =====================================================
    // STEREO
    // =====================================================

    private fun decodeStereoCameraFrame(
        json: JSONObject
    ) {

        if (
            isDecodingStereoFrame
        ) {

            return
        }


        isDecodingStereoFrame =
            true


        try {

            val bitmap =
                decodeCompressedImage(
                    json
                )
                    ?: return


            mainHandler.post {

                stereoBitmap =
                    bitmap
            }


        } catch (
            exception: Exception
        ) {

            println(
                "Błąd dekodowania stereo: " +
                        exception.message
            )


        } finally {

            isDecodingStereoFrame =
                false
        }
    }


    // =====================================================
    // MAPA GŁĘBI
    // =====================================================

    private fun decodeDepthCameraFrame(
        json: JSONObject
    ) {

        if (
            isDecodingDepthFrame
        ) {

            return
        }


        isDecodingDepthFrame =
            true


        try {

            val bitmap =
                decodeCompressedImage(
                    json
                )
                    ?: return


            mainHandler.post {

                depthBitmap =
                    bitmap
            }


        } catch (
            exception: Exception
        ) {

            println(
                "Błąd dekodowania mapy głębi: " +
                        exception.message
            )


        } finally {

            isDecodingDepthFrame =
                false
        }
    }


    // =====================================================
    // ARUCO DEBUG
    // =====================================================

    private fun decodeArucoCameraFrame(
        json: JSONObject
    ) {

        if (isDecodingArucoFrame) {
            return
        }

        isDecodingArucoFrame = true

        try {

            val bitmap =
                decodeCompressedImage(
                    json
                )
                    ?: return

            mainHandler.post {
                arucoBitmap = bitmap
            }

        } catch (
            exception: Exception
        ) {

            println(
                "Blad dekodowania ArUco debug: " +
                        exception.message
            )

        } finally {

            isDecodingArucoFrame = false
        }
    }


    // =====================================================
    // AI - WYKRYWANIE KOSTKI
    // =====================================================

    private fun decodeAiCameraFrame(
        json: JSONObject
    ) {

        if (
            isDecodingAiFrame
        ) {

            return
        }


        isDecodingAiFrame =
            true


        try {

            val bitmap =
                decodeCompressedImage(
                    json
                )
                    ?: return


            mainHandler.post {

                aiBitmap =
                    bitmap
            }


        } catch (
            exception: Exception
        ) {

            println(
                "Błąd dekodowania obrazu AI: " +
                        exception.message
            )


        } finally {

            isDecodingAiFrame =
                false
        }
    }


    // =====================================================
    // WSPÓLNE DEKODOWANIE COMPRESSED IMAGE
    // =====================================================

    private fun decodeCompressedImage(
        json: JSONObject
    ): Bitmap? {

        val rosMessage =
            json.optJSONObject(
                "msg"
            )
                ?: return null


        val encodedImage =
            rosMessage.optString(
                "data"
            )


        if (
            encodedImage.isBlank()
        ) {

            return null
        }


        val imageBytes =
            Base64.decode(
                encodedImage,
                Base64.NO_WRAP
            )


        return BitmapFactory.decodeByteArray(
            imageBytes,
            0,
            imageBytes.size
        )
    }


    // =====================================================
    // ZAMYKANIE
    // =====================================================

    override fun onCleared() {

        if (
            isConnected
            &&
            buzzerEnabled
        ) {

            rosBridgeClient
                .publishManipulatorCommand(
                    "8"
                )
        }


        stopAutonomousPreview()


        rosBridgeClient.shutdown()


        super.onCleared()
    }
}