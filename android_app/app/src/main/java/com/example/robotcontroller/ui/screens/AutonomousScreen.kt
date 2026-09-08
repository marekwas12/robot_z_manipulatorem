package com.example.robotcontroller.ui.screens

import android.graphics.Bitmap

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width

import androidx.compose.foundation.shape.RoundedCornerShape

import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Text

import androidx.compose.runtime.Composable

import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp


private val MenuButtonColor =
    Color(0xFF506AA3)

private val AutonomousColor =
    Color(0xFF2E7D32)

private val DisabledColor =
    Color(0xFFBDBDBD)


// =============================================================
// MENU TRYBU AUTONOMICZNEGO
//
// TEN EKRAN JEST PIONOWY.
// =============================================================

@Composable
fun AutonomousMenuScreen(

    onStereoClick: () -> Unit,

    onDepthClick: () -> Unit,

    onAiClick: () -> Unit,

    onArucoClick: () -> Unit,

    onCubePickClick: () -> Unit,

    onAutonomousDriveClick: () -> Unit,

    onBackClick: () -> Unit

) {

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(
                horizontal = 20.dp,
                vertical = 18.dp
            ),

        horizontalAlignment =
        Alignment.CenterHorizontally
    ) {

        Text(
            text = "Tryb autonomiczny",
            fontSize = 28.sp,
            fontWeight = FontWeight.Bold
        )

        Spacer(
            modifier = Modifier.height(22.dp)
        )

        // =====================================================
        // KAMERY
        // =====================================================

        Text(
            text = "Kamery",
            fontSize = 18.sp,
            fontWeight = FontWeight.Bold
        )

        Spacer(
            modifier = Modifier.height(12.dp)
        )

        AutonomousMenuButton(
            text = "Obie kamery",
            onClick = onStereoClick
        )

        Spacer(
            modifier = Modifier.height(10.dp)
        )

        AutonomousMenuButton(
            text = "Mapa głębi",
            onClick = onDepthClick
        )

        Spacer(
            modifier = Modifier.height(10.dp)
        )

        AutonomousMenuButton(
            text = "Wykrywanie kostki",
            onClick = onAiClick
        )

        Spacer(
            modifier = Modifier.height(10.dp)
        )

        AutonomousMenuButton(
            text = "ArUco",
            onClick = onArucoClick
        )

        Spacer(
            modifier = Modifier.height(26.dp)
        )

        // =====================================================
        // FUNKCJE AUTONOMICZNE
        // =====================================================

        Text(
            text = "Funkcje autonomiczne",
            fontSize = 18.sp,
            fontWeight = FontWeight.Bold
        )

        Spacer(
            modifier = Modifier.height(12.dp)
        )

        AutonomousMenuButton(
            text = "Podnoszenie kostki",
            onClick = onCubePickClick
        )

        Spacer(
            modifier = Modifier.height(10.dp)
        )

        AutonomousMenuButton(
            text = "Jazda półautonomiczna",
            onClick = onAutonomousDriveClick
        )

        Spacer(
            modifier = Modifier.weight(1f)
        )

        Button(
            onClick = onBackClick,

            modifier = Modifier
                .fillMaxWidth()
                .height(52.dp),

            shape = RoundedCornerShape(15.dp),

            colors = ButtonDefaults.buttonColors(
                containerColor = MenuButtonColor
            )
        ) {

            Text(
                text = "Wróć",
                fontSize = 14.sp,
                fontWeight = FontWeight.Bold
            )
        }
    }
}


// =============================================================
// PRZYCISK MENU
// =============================================================

@Composable
private fun AutonomousMenuButton(

    text: String,

    onClick: () -> Unit

) {

    Button(
        onClick =
        onClick,

        modifier =
        Modifier
            .fillMaxWidth()
            .height(
                54.dp
            ),

        shape =
        RoundedCornerShape(
            15.dp
        ),

        colors =
        ButtonDefaults
            .buttonColors(

                containerColor =
                AutonomousColor
            )
    ) {

        Text(
            text =
            text,

            fontSize =
            16.sp,

            fontWeight =
            FontWeight.Bold
        )
    }
}


// =============================================================
// PODNIESIENIE KOSTKI
//
// Kamera po lewej = /cube_aruco/debug/compressed.
// To dokładnie ten tor ArUco, z którego korzysta cube_pick_node.
//
// PODNIEŚ KOSTKĘ -> /cube_pick/command = full
// DO BAGAŻNIKA   -> /cube_place/command = start
// =============================================================

@Composable
fun CubePickScreen(

    arucoBitmap: Bitmap?,

    arucoDetected: Boolean,

    arucoCubeId: Int?,

    arucoCubeX: Float?,
    arucoCubeY: Float?,
    arucoCubeDistanceM: Float?,

    arucoCubeYawDeg: Float?,

    arucoQuality: Float?,

    cubePickStatus: String,

    cubePlaceStatus: String,

    onPickClick: () -> Unit,

    onPlaceClick: () -> Unit,

    onBackClick: () -> Unit

) {

    Row(
        modifier = Modifier
            .fillMaxSize()
            .background(Color.Black)
    ) {

        Box(
            modifier = Modifier
                .weight(1.45f)
                .fillMaxHeight()
                .background(Color.Black),

            contentAlignment = Alignment.Center
        ) {

            if (arucoBitmap != null) {

                Image(
                    bitmap = arucoBitmap.asImageBitmap(),
                    contentDescription = "Kamera cube pick ArUco",

                    modifier = Modifier
                        .fillMaxSize()
                        .graphicsLayer {
                            // Podgląd ArUco był wcześniej poprawny przy 180°.
                            rotationZ = 180f
                        },

                    contentScale = ContentScale.Fit
                )

            } else {

                Text(
                    text = "Czekam na kamerę ArUco...",
                    color = Color.White,
                    fontSize = 16.sp
                )
            }
        }

        Column(
            modifier = Modifier
                .width(310.dp)
                .fillMaxHeight()
                .background(Color(0xFF171717))
                .padding(14.dp),

            horizontalAlignment = Alignment.CenterHorizontally
        ) {

            Text(
                text = if (arucoDetected) {
                    "KOSTKA WYKRYTA | ArUco ID: ${arucoCubeId ?: "---"}"
                } else {
                    "BRAK WYKRYCIA KOSTKI"
                },
                color = if (arucoDetected) {
                    Color(0xFFB9F6CA)
                } else {
                    Color(0xFFFF8A80)
                },
                fontSize = 13.sp,
                fontWeight = FontWeight.Bold,
                textAlign = TextAlign.Center
            )

            Spacer(modifier = Modifier.height(6.dp))

            Text(
                text = if (
                    arucoCubeX != null &&
                    arucoCubeY != null &&
                    arucoCubeDistanceM != null
                ) {
                    "X=%+.3f  Y=%+.3f  Z=%.3f m".format(
                        arucoCubeX,
                        arucoCubeY,
                        arucoCubeDistanceM
                    )
                } else {
                    "XYZ: ---"
                },
                color = Color(0xFF80D8FF),
                fontSize = 12.sp,
                textAlign = TextAlign.Center
            )

            Text(
                text = if (arucoCubeYawDeg != null) {
                    "Yaw: %+.1f°".format(arucoCubeYawDeg)
                } else {
                    "Yaw: ---"
                },
                color = Color(0xFFFFF59D),
                fontSize = 12.sp
            )

            Text(
                text = if (arucoQuality != null) {
                    "Jakość: %.2f".format(arucoQuality)
                } else {
                    "Jakość: ---"
                },
                color = Color.LightGray,
                fontSize = 12.sp
            )

            Spacer(modifier = Modifier.height(14.dp))

            Button(
                onClick = onPickClick,
                modifier = Modifier
                    .fillMaxWidth(0.88f)
                    .height(42.dp),
                shape = RoundedCornerShape(13.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = AutonomousColor
                )
            ) {
                Text(
                    text = "PODNIEŚ KOSTKĘ",
                    fontSize = 13.sp,
                    fontWeight = FontWeight.Bold
                )
            }

            Spacer(modifier = Modifier.height(8.dp))

            Button(
                onClick = onPlaceClick,
                modifier = Modifier
                    .fillMaxWidth(0.88f)
                    .height(42.dp),
                shape = RoundedCornerShape(13.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = Color(0xFF1565C0)
                )
            ) {
                Text(
                    text = "DO BAGAŻNIKA",
                    fontSize = 13.sp,
                    fontWeight = FontWeight.Bold
                )
            }

            Spacer(modifier = Modifier.weight(1f))

            Button(
                onClick = onBackClick,
                modifier = Modifier
                    .fillMaxWidth()
                    .height(50.dp),
                shape = RoundedCornerShape(15.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = MenuButtonColor
                )
            ) {
                Text(
                    text = "Wróć",
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}


// =============================================================
// EKRAN AUTONOMICZNEGO ZBIERANIA
//
// POZIOMO:
// - kamera po lewej
// - START / STOP
// - 6 x HC-SR04 na żywo
// =============================================================

@Composable
fun AutonomousDriveScreen(

    depthBitmap: Bitmap?,

    aiBitmap: Bitmap?,

    status: String,

    aiDriveDetected: Boolean,

    aiDriveX: Float?,
    aiDriveY: Float?,
    aiDriveDistanceM: Float?,
    aiDriveBearingDeg: Float?,

    cubesLoaded: Int,

    // Dane BEZPOŚREDNIO z aruco_cube_detector / cube_pick pipeline.
    arucoDetected: Boolean,

    arucoCubeId: Int?,

    arucoCubeX: Float?,
    arucoCubeY: Float?,
    arucoCubeDistanceM: Float?,

    // Kąt do środka obrazu: atan2(X, Z), tylko do jazdy.
    arucoBearingDeg: Float?,

    // Prawdziwy yaw kostki używany przez cube_pick do J5.
    arucoCubeYawDeg: Float?,

    arucoQuality: Float?,

    frontLeftCm: Float?,
    frontCenterCm: Float?,
    frontRightCm: Float?,

    leftCm: Float?,
    rightCm: Float?,

    rearCm: Float?,

    onStartClick: () -> Unit,

    onStopClick: () -> Unit,

    onBackClick: () -> Unit

) {

    Row(
        modifier =
        Modifier
            .fillMaxSize()
            .background(
                Color.Black
            )
    ) {

        // =====================================================
        // DEPTH + ARUCO
        //
        // ArUco debug pochodzi z /cube_aruco/debug/compressed.
        // To jest TEN SAM detector, którego pozycję/yaw czyta cube_pick.
        // =====================================================

        Box(
            modifier =
            Modifier
                .weight(
                    1.35f
                )
                .fillMaxHeight()
                .background(
                    Color.Black
                ),

            contentAlignment =
            Alignment.Center
        ) {

            if (depthBitmap != null) {

                Image(
                    bitmap =
                    depthBitmap
                        .asImageBitmap(),

                    contentDescription =
                    "Mapa głębi",

                    modifier =
                    Modifier
                        .fillMaxSize(),

                    contentScale =
                    ContentScale.Fit
                )

            } else {

                Text(
                    text =
                    "Czekam na mapę głębi...",

                    color =
                    Color.White,

                    fontSize =
                    15.sp
                )
            }


            if (aiBitmap != null) {

                Image(
                    bitmap =
                    aiBitmap
                        .asImageBitmap(),

                    contentDescription =
                    "AI kostka podczas jazdy",

                    modifier =
                    Modifier
                        .fillMaxSize()
                        .graphicsLayer {
                            alpha = 0.62f
                        },

                    contentScale =
                    ContentScale.Fit
                )
            }


        }


        // =====================================================
        // PANEL - PRAWA STRONA
        // =====================================================

        Column(
            modifier =
            Modifier
                .width(
                    315.dp
                )
                .fillMaxHeight()
                .background(
                    Color(0xFF171717)
                )
                .padding(
                    horizontal =
                    5.dp,

                    vertical =
                    2.dp
                )
        ) {


            Text(
                text =
                "Status: $status",

                modifier =
                Modifier.fillMaxWidth(),

                color =
                Color(0xFFB9F6CA),

                fontSize =
                8.sp,

                maxLines = 2,

                textAlign =
                TextAlign.Center
            )


            Text(
                text =
                "Kostki w bagażniku: $cubesLoaded / 4",

                modifier =
                Modifier.fillMaxWidth(),

                color =
                Color(0xFFFFF59D),

                fontSize =
                8.sp,

                fontWeight =
                FontWeight.Bold,

                textAlign =
                TextAlign.Center
            )


            // =================================================
            // JAZDA - AI
            // =================================================

            Text(
                text =
                if (
                    aiDriveDetected
                    &&
                    aiDriveDistanceM != null
                ) {
                    "AI: WYKRYTA | Z=%.3f m"
                        .format(aiDriveDistanceM)
                } else {
                    "AI: ---"
                },

                modifier =
                Modifier.fillMaxWidth(),

                color =
                if (aiDriveDetected) {
                    Color(0xFFB9F6CA)
                } else {
                    Color.Gray
                },

                fontSize =
                9.sp,

                fontWeight =
                FontWeight.Bold,

                textAlign =
                TextAlign.Center
            )


            Text(
                text =
                if (aiDriveBearingDeg != null) {

                    val side =
                        when {
                            aiDriveBearingDeg >= -6.5f
                                    && aiDriveBearingDeg <= 6.5f ->
                                "ŚRODEK"

                            aiDriveBearingDeg > 6.5f
                                    && aiDriveBearingDeg <= 10.5f ->
                                "TROCHĘ PRAWO"

                            aiDriveBearingDeg > 10.5f ->
                                "PRAWO"

                            aiDriveBearingDeg < -6.5f
                                    && aiDriveBearingDeg >= -10.5f ->
                                "TROCHĘ LEWO"

                            else ->
                                "LEWO"
                        }

                    "AI kąt: %+.1f° | %s"
                        .format(
                            aiDriveBearingDeg,
                            side
                        )

                } else {

                    "AI kąt: ---"
                },

                modifier =
                Modifier.fillMaxWidth(),

                color =
                Color(0xFFFFF59D),

                fontSize =
                9.sp,

                fontWeight =
                FontWeight.Bold,

                textAlign =
                TextAlign.Center
            )


            // =================================================
            // DANE DOKŁADNIE Z /cube_aruco/*
            // =================================================

            Text(
                text =
                "ArUco: " +
                        if (arucoDetected) {
                            "WYKRYTA"
                        } else {
                            "---"
                        }
                        +
                        "   |   ID: "
                        +
                        (
                                arucoCubeId
                                    ?.toString()
                                    ?: "---"
                                ),

                modifier =
                Modifier
                    .fillMaxWidth()
                    .background(
                        Color(0xFF252525)
                    )
                    .padding(
                        vertical = 2.dp,
                        horizontal = 4.dp
                    ),

                color =
                if (arucoDetected) {
                    Color(0xFFB9F6CA)
                } else {
                    Color.Gray
                },

                fontSize =
                8.sp,

                fontWeight =
                FontWeight.Bold,

                textAlign =
                TextAlign.Center
            )


            Text(
                text =
                if (
                    arucoCubeX != null
                    &&
                    arucoCubeY != null
                    &&
                    arucoCubeDistanceM != null
                ) {

                    "ArUco XYZ: X=%+.3f  Y=%+.3f  Z=%.3f m"
                        .format(
                            arucoCubeX,
                            arucoCubeY,
                            arucoCubeDistanceM
                        )

                } else {

                    "ArUco XYZ: ---"
                },

                modifier =
                Modifier
                    .fillMaxWidth()
                    .background(
                        Color(0xFF252525)
                    )
                    .padding(
                        vertical = 2.dp,
                        horizontal = 4.dp
                    ),

                color =
                Color(0xFF80D8FF),

                fontSize =
                8.sp,

                fontWeight =
                FontWeight.Bold,

                textAlign =
                TextAlign.Center
            )


            Text(
                text =
                if (arucoBearingDeg != null) {

                    val side =
                        when {

                            arucoBearingDeg >= -6.5f
                                    && arucoBearingDeg <= 6.5f ->
                                "ŚRODEK"

                            arucoBearingDeg > 6.5f
                                    && arucoBearingDeg <= 10.5f ->
                                "TROCHĘ PRAWO"

                            arucoBearingDeg > 10.5f ->
                                "PRAWO"

                            arucoBearingDeg < -6.5f
                                    && arucoBearingDeg >= -10.5f ->
                                "TROCHĘ LEWO"

                            else ->
                                "LEWO"
                        }

                    "ArUco kąt: %+.1f°   |   %s"
                        .format(
                            arucoBearingDeg,
                            side
                        )

                } else {

                    "ArUco kąt: ---"
                },

                modifier =
                Modifier
                    .fillMaxWidth()
                    .background(
                        Color(0xFF252525)
                    )
                    .padding(
                        vertical = 2.dp,
                        horizontal = 4.dp
                    ),

                color =
                Color(0xFFFFF59D),

                fontSize =
                8.sp,

                fontWeight =
                FontWeight.Bold,

                textAlign =
                TextAlign.Center
            )


            Text(
                text =
                "Yaw cube_pick: "
                        +
                        (
                                arucoCubeYawDeg
                                    ?.let {
                                        "%+.1f°".format(it)
                                    }
                                    ?: "---"
                                )
                        +
                        "   |   Q: "
                        +
                        (
                                arucoQuality
                                    ?.let {
                                        "%.2f".format(it)
                                    }
                                    ?: "---"
                                ),

                modifier =
                Modifier.fillMaxWidth(),

                color =
                Color(0xFFFFCC80),

                fontSize =
                8.sp,

                textAlign =
                TextAlign.Center
            )


            Spacer(
                modifier =
                Modifier.height(
                    2.dp
                )
            )


            // START / STOP / WRÓĆ
            Row(
                modifier =
                Modifier.fillMaxWidth(),

                horizontalArrangement =
                Arrangement.spacedBy(
                    6.dp
                )
            ) {

                Button(
                    onClick =
                    onStartClick,

                    // Cleanup nie używa HC-SR04 do logiki jazdy.
                    // Czujniki można włączyć niezależnie do podglądu.
                    enabled =
                    true,

                    modifier =
                    Modifier
                        .weight(
                            1f
                        )
                        .height(
                            34.dp
                        ),

                    shape =
                    RoundedCornerShape(
                        9.dp
                    ),

                    colors =
                    ButtonDefaults
                        .buttonColors(
                            containerColor =
                            AutonomousColor
                        )
                ) {

                    Text(
                        text =
                        "START",

                        fontSize =
                        8.sp,

                        fontWeight =
                        FontWeight.Bold
                    )
                }


                Button(
                    onClick =
                    onStopClick,

                    modifier =
                    Modifier
                        .weight(
                            1f
                        )
                        .height(
                            34.dp
                        ),

                    shape =
                    RoundedCornerShape(
                        9.dp
                    ),

                    colors =
                    ButtonDefaults
                        .buttonColors(
                            containerColor =
                            Color(0xFFC62828)
                        )
                ) {

                    Text(
                        text =
                        "STOP",

                        fontSize =
                        8.sp,

                        fontWeight =
                        FontWeight.Bold
                    )
                }


                Button(
                    onClick =
                    onBackClick,

                    modifier =
                    Modifier
                        .weight(
                            1f
                        )
                        .height(
                            34.dp
                        ),

                    shape =
                    RoundedCornerShape(
                        9.dp
                    ),

                    colors =
                    ButtonDefaults
                        .buttonColors(
                            containerColor =
                            MenuButtonColor
                        )
                ) {

                    Text(
                        text =
                        "WRÓĆ",

                        fontSize =
                        8.sp,

                        fontWeight =
                        FontWeight.Bold
                    )
                }
            }


            Spacer(
                modifier =
                Modifier.height(
                    3.dp
                )
            )


            // HC-SR04 pozostają tylko dodatkowym podglądem.
            Text(
                text =
                "HC-SR04 [cm]",

                modifier =
                Modifier.fillMaxWidth(),

                color =
                Color(0xFFB9F6CA),

                fontSize =
                8.sp,

                fontWeight =
                FontWeight.Bold,

                textAlign =
                TextAlign.Center
            )


            Row(
                modifier =
                Modifier.fillMaxWidth(),

                horizontalArrangement =
                Arrangement.spacedBy(
                    8.dp
                )
            ) {

                CompactSensorColumn(
                    modifier =
                    Modifier.weight(
                        1f
                    ),

                    item1Label =
                    "Przód L",

                    item1Value =
                    frontLeftCm,

                    item2Label =
                    "Przód C",

                    item2Value =
                    frontCenterCm,

                    item3Label =
                    "Przód P",

                    item3Value =
                    frontRightCm
                )


                CompactSensorColumn(
                    modifier =
                    Modifier.weight(
                        1f
                    ),

                    item1Label =
                    "Lewy",

                    item1Value =
                    leftCm,

                    item2Label =
                    "Prawy",

                    item2Value =
                    rightCm,

                    item3Label =
                    "Tył",

                    item3Value =
                    rearCm
                )
            }
        }
    }
}



@Composable
private fun CompactSensorColumn(

    modifier: Modifier,

    item1Label: String,

    item1Value: Float?,

    item2Label: String,

    item2Value: Float?,

    item3Label: String,

    item3Value: Float?

) {

    Column(
        modifier =
        modifier
    ) {

        CompactSensorRow(
            label =
            item1Label,

            value =
            item1Value
        )


        CompactSensorRow(
            label =
            item2Label,

            value =
            item2Value
        )


        CompactSensorRow(
            label =
            item3Label,

            value =
            item3Value
        )
    }
}


@Composable
private fun CompactSensorRow(

    label: String,

    value: Float?

) {

    val valueText =
        if (
            value == null
            ||
            value < 0f
        ) {

            "---"

        } else {

            String.format(
                "%.1f",
                value
            )
        }


    Row(
        modifier =
        Modifier
            .fillMaxWidth()
            .height(
                22.dp
            )
            .padding(
                horizontal =
                5.dp
            ),

        verticalAlignment =
        Alignment.CenterVertically,

        horizontalArrangement =
        Arrangement.SpaceBetween
    ) {

        Text(
            text =
            label,

            color =
            Color.White,

            fontSize =
            9.sp
        )


        Text(
            text =
            valueText,

            color =
            Color(0xFFFFF59D),

            fontSize =
            10.sp,

            fontWeight =
            FontWeight.Bold
        )
    }
}



@Composable
private fun SensorRow(

    label: String,

    value: Float?

) {

    val text =
        if (
            value == null
            ||
            value < 0f
        ) {

            "---"

        } else {

            String.format(
                "%.1f",
                value
            )
        }


    Row(
        modifier =
        Modifier
            .fillMaxWidth()
            .padding(
                vertical =
                3.dp
            ),

        horizontalArrangement =
        Arrangement.SpaceBetween
    ) {

        Text(
            text =
            label,

            color =
            Color.White,

            fontSize =
            15.sp
        )


        Text(
            text =
            text,

            color =
            Color(0xFFFFF59D),

            fontSize =
            16.sp,

            fontWeight =
            FontWeight.Bold
        )
    }
}



// =============================================================
// EKRAN STEREO
//
// TEN EKRAN JEST POZIOMY.
// =============================================================

@Composable
fun StereoCameraScreen(

    stereoBitmap: Bitmap?,

    stereoEnabled: Boolean,

    onStereoClick: () -> Unit,

    onBackClick: () -> Unit

) {

    Row(
        modifier = Modifier
            .fillMaxSize()
            .background(
                Color.Black
            )
    ) {


        // =====================================================
        // DUŻY OBRAZ
        // =====================================================

        Box(
            modifier = Modifier
                .weight(
                    1f
                )
                .fillMaxHeight(),

            contentAlignment =
            Alignment.Center
        ) {


            if (
                stereoEnabled
                &&
                stereoBitmap != null
            ) {

                Image(
                    bitmap =
                    stereoBitmap
                        .asImageBitmap(),

                    contentDescription =
                    "Kamery stereowizyjne",

                    modifier =
                    Modifier
                        .fillMaxSize(),

                    contentScale =
                    ContentScale.Fit
                )


            } else {

                Text(
                    text =
                    if (
                        stereoEnabled
                    ) {
                        "Czekam na obraz kamer..."
                    } else {
                        "Kamery wyłączone"
                    },

                    color =
                    Color.White,

                    fontSize =
                    17.sp
                )
            }
        }


        // =====================================================
        // PANEL BOCZNY
        // =====================================================

        CameraSidePanel(

            enabled =
            stereoEnabled,

            activeText =
            "Kamery aktywne",

            inactiveText =
            "Kamery wyłączone",

            buttonOnText =
            "Wyłącz",

            buttonOffText =
            "Włącz kamery",

            onToggle =
            onStereoClick,

            onBackClick =
            onBackClick
        )
    }
}


// =============================================================
// EKRAN MAPY GŁĘBI
//
// TEN EKRAN JEST POZIOMY.
// =============================================================

@Composable
fun DepthMapScreen(

    depthBitmap: Bitmap?,

    depthEnabled: Boolean,

    onDepthClick: () -> Unit,

    onBackClick: () -> Unit

) {

    Row(
        modifier = Modifier
            .fillMaxSize()
            .background(
                Color.Black
            )
    ) {


        // =====================================================
        // DUŻA MAPA
        // =====================================================

        Box(
            modifier = Modifier
                .weight(
                    1f
                )
                .fillMaxHeight(),

            contentAlignment =
            Alignment.Center
        ) {


            if (
                depthEnabled
                &&
                depthBitmap != null
            ) {

                Image(
                    bitmap =
                    depthBitmap
                        .asImageBitmap(),

                    contentDescription =
                    "Mapa głębi",

                    modifier =
                    Modifier
                        .fillMaxSize(),

                    contentScale =
                    ContentScale.Fit
                )


            } else {

                Text(
                    text =
                    if (
                        depthEnabled
                    ) {
                        "Czekam na mapę głębi..."
                    } else {
                        "Mapa głębi wyłączona"
                    },

                    color =
                    Color.White,

                    fontSize =
                    17.sp,

                    textAlign =
                    TextAlign.Center
                )
            }
        }


        // =====================================================
        // PANEL BOCZNY
        // =====================================================

        CameraSidePanel(

            enabled =
            depthEnabled,

            activeText =
            "Głębia aktywna",

            inactiveText =
            "Głębia wyłączona",

            buttonOnText =
            "Wyłącz głębię",

            buttonOffText =
            "Włącz głębię",

            onToggle =
            onDepthClick,

            onBackClick =
            onBackClick
        )
    }
}


// =============================================================
// EKRAN WYKRYWANIA KOSTKI AI
//
// TEN EKRAN JEST POZIOMY.
// =============================================================

@Composable
fun AiCubeDetectionScreen(

    aiBitmap: Bitmap?,

    aiEnabled: Boolean,

    onAiClick: () -> Unit,

    onBackClick: () -> Unit

) {

    Row(
        modifier = Modifier
            .fillMaxSize()
            .background(
                Color.Black
            )
    ) {

        Box(
            modifier = Modifier
                .weight(
                    1f
                )
                .fillMaxHeight(),

            contentAlignment =
            Alignment.Center
        ) {

            if (
                aiEnabled
                &&
                aiBitmap != null
            ) {

                Image(
                    bitmap =
                    aiBitmap
                        .asImageBitmap(),

                    contentDescription =
                    "Wykrywanie kostki AI",

                    modifier =
                    Modifier
                        .fillMaxSize(),

                    contentScale =
                    ContentScale.Fit
                )

            } else {

                Text(
                    text =
                    if (aiEnabled) {
                        "Czekam na obraz AI..."
                    } else {
                        "Podgląd AI wyłączony"
                    },

                    color =
                    Color.White,

                    fontSize =
                    17.sp,

                    textAlign =
                    TextAlign.Center
                )
            }
        }


        CameraSidePanel(

            enabled =
            aiEnabled,

            activeText =
            "AI aktywne",

            inactiveText =
            "AI wyłączone",

            buttonOnText =
            "Wyłącz AI",

            buttonOffText =
            "Włącz AI",

            onToggle =
            onAiClick,

            onBackClick =
            onBackClick
        )
    }
}


// =============================================================
// WSPÓLNY PANEL DLA STEREO / DEPTH
// =============================================================

@Composable
private fun CameraSidePanel(

    enabled: Boolean,

    activeText: String,

    inactiveText: String,

    buttonOnText: String,

    buttonOffText: String,

    onToggle: () -> Unit,

    onBackClick: () -> Unit

) {

    Column(
        modifier = Modifier
            .width(
                190.dp
            )
            .fillMaxHeight()
            .background(
                Color(
                    0xFF151515
                )
            )
            .padding(
                12.dp
            ),

        horizontalAlignment =
        Alignment.CenterHorizontally
    ) {


        Button(
            onClick =
            onToggle,

            modifier =
            Modifier
                .fillMaxWidth()
                .height(
                    50.dp
                )
        ) {

            Text(
                text =
                if (
                    enabled
                ) {
                    buttonOnText
                } else {
                    buttonOffText
                },

                fontWeight =
                FontWeight.Bold
            )
        }


        Spacer(
            modifier =
            Modifier.height(
                12.dp
            )
        )


        Text(
            text =
            if (
                enabled
            ) {
                activeText
            } else {
                inactiveText
            },

            color =
            if (
                enabled
            ) {
                Color(
                    0xFF66BB6A
                )
            } else {
                Color.Gray
            },

            fontSize =
            15.sp,

            textAlign =
            TextAlign.Center
        )


        Spacer(
            modifier =
            Modifier.weight(
                1f
            )
        )


        Button(
            onClick =
            onBackClick,

            modifier =
            Modifier
                .fillMaxWidth()
                .height(
                    50.dp
                )
        ) {

            Text(
                text = "Wróć",

                fontWeight =
                FontWeight.Bold
            )
        }
    }
}


// =============================================================
// EKRAN ARUCO - PODGLĄD + DANE DETEKTORA
// =============================================================

@Composable
fun ArucoCubeDetectionScreen(

    arucoBitmap: Bitmap?,
    arucoEnabled: Boolean,

    arucoDetected: Boolean,
    arucoCubeId: Int?,

    arucoCubeX: Float?,
    arucoCubeY: Float?,
    arucoCubeDistanceM: Float?,

    arucoBearingDeg: Float?,
    arucoCubeYawDeg: Float?,
    arucoQuality: Float?,

    onBackClick: () -> Unit

) {

    Row(
        modifier =
        Modifier
            .fillMaxSize()
            .background(
                Color(0xFF101318)
            )
            .padding(
                12.dp
            )
    ) {

        Box(
            modifier =
            Modifier
                .weight(1.7f)
                .fillMaxHeight()
                .background(
                    Color.Black,
                    RoundedCornerShape(
                        12.dp
                    )
                ),

            contentAlignment =
            Alignment.Center
        ) {

            if (
                arucoEnabled
                &&
                arucoBitmap != null
            ) {

                Image(
                    bitmap =
                    arucoBitmap
                        .asImageBitmap(),

                    contentDescription =
                    "Podgląd ArUco",

                    modifier =
                    Modifier
                        .fillMaxSize()
                        .graphicsLayer {
                            rotationZ = 180f
                        },

                    contentScale =
                    ContentScale.Fit
                )

            } else {

                Text(
                    text =
                    if (arucoEnabled) {
                        "Czekam na obraz ArUco..."
                    } else {
                        "Podgląd ArUco wyłączony"
                    },

                    color =
                    Color.White,

                    fontSize =
                    18.sp,

                    textAlign =
                    TextAlign.Center
                )
            }
        }


        Spacer(
            modifier =
            Modifier.width(
                14.dp
            )
        )


        Column(
            modifier =
            Modifier
                .weight(1.0f)
                .fillMaxHeight()
                .background(
                    Color(0xFF1A1F27),
                    RoundedCornerShape(
                        12.dp
                    )
                )
                .padding(
                    14.dp
                ),

            verticalArrangement =
            Arrangement.spacedBy(
                8.dp
            )
        ) {

            Text(
                text =
                "ArUco - publikowane dane",

                color =
                Color.White,

                fontSize =
                19.sp,

                fontWeight =
                FontWeight.Bold
            )


            Text(
                text =
                if (arucoDetected) {
                    "WYKRYTO KOSTKĘ"
                } else {
                    "BRAK KOSTKI"
                },

                color =
                if (arucoDetected) {
                    Color(0xFFB9F6CA)
                } else {
                    Color(0xFFFF8A80)
                },

                fontSize =
                15.sp,

                fontWeight =
                FontWeight.Bold
            )


            Text(
                text =
                "ID: " +
                        (
                                arucoCubeId
                                    ?.toString()
                                    ?: "---"
                                ),

                color =
                Color.White,

                fontSize =
                14.sp
            )


            Text(
                text =
                "X: " +
                        (
                                arucoCubeX
                                    ?.let {
                                        "%+.4f m".format(it)
                                    }
                                    ?: "---"
                                ),

                color =
                Color.White,

                fontSize =
                14.sp
            )


            Text(
                text =
                "Y: " +
                        (
                                arucoCubeY
                                    ?.let {
                                        "%+.4f m".format(it)
                                    }
                                    ?: "---"
                                ),

                color =
                Color.White,

                fontSize =
                14.sp
            )


            Text(
                text =
                "Z: " +
                        (
                                arucoCubeDistanceM
                                    ?.let {
                                        "%.4f m".format(it)
                                    }
                                    ?: "---"
                                ),

                color =
                Color.White,

                fontSize =
                14.sp,

                fontWeight =
                FontWeight.Bold
            )


            Text(
                text =
                "Kąt L/P: " +
                        (
                                arucoBearingDeg
                                    ?.let {
                                        "%+.1f°".format(it)
                                    }
                                    ?: "---"
                                ),

                color =
                Color(0xFFFFF59D),

                fontSize =
                14.sp
            )


            Text(
                text =
                "Yaw: " +
                        (
                                arucoCubeYawDeg
                                    ?.let {
                                        "%+.1f°".format(it)
                                    }
                                    ?: "---"
                                ),

                color =
                Color.White,

                fontSize =
                14.sp
            )


            Text(
                text =
                "Q: " +
                        (
                                arucoQuality
                                    ?.let {
                                        "%.3f".format(it)
                                    }
                                    ?: "---"
                                ),

                color =
                Color.White,

                fontSize =
                14.sp
            )


            Spacer(
                modifier =
                Modifier.weight(
                    1f
                )
            )


            Button(
                onClick =
                onBackClick,

                modifier =
                Modifier
                    .fillMaxWidth()
                    .height(
                        48.dp
                    ),

                colors =
                ButtonDefaults.buttonColors(
                    containerColor =
                    MenuButtonColor
                )
            ) {

                Text(
                    text =
                    "Wróć",

                    fontWeight =
                    FontWeight.Bold
                )
            }
        }
    }
}