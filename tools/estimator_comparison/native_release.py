"""Adapt the external replay boundary to the pinned stable firmware APIs."""

SOURCE_PINS = {
    "px4": "d6f12ad1c4f70ad3230afd7d86e971421e02fef4",
    "ardupilot": "dbe792162d06cab66c3475fd5556bf7a120f119e",
}


def px4_release_adapter(text, release, replace):
    if release != "v1.17.0":
        raise ValueError("Unsupported PX4 replay API")
    replacements = (
        (
            "\tFusionControl *fc = ekf.getFusionControlHandle();\n"
            "\tfc->mag.enabled = !mag.empty(); fc->baro.enabled = !baro.empty();\n"
            "\tfc->gps.enabled = use_gps; fc->of.enabled = use_flow; fc->rng.enabled = use_flow;\n",
            "",
        ),
        ("\tekf.set_armed_status(false);\n", ""),
        ("\t\tekf.set_armed_status(in_air);\n", ""),
        (" armed=%d", ""),
        (" (int)f.armed,", ""),
    )
    for before, after in replacements:
        text = replace(text, before, after)
    return text
