package com.example.robotcontroller.models


enum class AppScreen {

    HOME,

    DRIVE,

    MANIPULATOR,

    // Główne pionowe menu trybu autonomicznego
    AUTONOMOUS,

    // Jazda / autonomiczne zbieranie
    AUTONOMOUS_DRIVE,

    // Poziomy podgląd dwóch kamer
    AUTONOMOUS_STEREO,

    // Poziomy podgląd mapy głębi
    AUTONOMOUS_DEPTH,

    // Poziomy podgląd wykrywania kostki przez AI
    AUTONOMOUS_AI,

    // Poziomy podgląd ArUco + dane publikowane przez detector
    AUTONOMOUS_ARUCO,

    // Osobny ekran ręcznego uruchomienia cube_pick_node
    // + podgląd tej samej kamery ArUco, z której korzysta cube_pick.
    AUTONOMOUS_PICK
}