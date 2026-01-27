# Garage Door Controller (for Controllino Maxi + MQTT & Home Assistant Discovery)

DoorControllerMaxi is for controlling a garage door motor using a **Controllino Maxi**, with:

- **MQTT control + state publishing**
- **Home Assistant MQTT Discovery** (cover + buttons + optional binary sensors)
- **Local wall-button input**
- **Safety logic**
- A built-in **Web UI** for manual control and status.

---

## Features

### Home Assistant integration (MQTT Discovery)
Publishes discovery configs for:
- **Cover** entity: open/close/stop with state (`open`, `closed`, `opening`, `closing`, `stopped`)
- **Buttons**: Open / Close / Stop / Toggle
- **Binary sensors** (optional): Closed endstop, Open endstop, Obstruction (light curtain)

### Control inputs
- **MQTT** command topic: `OPEN`, `CLOSE`, `STOP`, `TOGGLE`
- **Local button** on `CONTROLLINO_A9`

### Outputs
- **Power relay** (motor enable)
- **Direction relay**
- **Status light**: blinks while moving, solid ON when idle
- **Locked/closed indicator**: ON when closed endstop active (if enabled)

### Safety / reliability
- Stops closing if **light curtain triggers**
- Stops when endstop reached (if configured)
- Stops after `DOOR_MAX_MOVEMENT_MS` timeout

---

> ⚠️ Don't forget, this project drives physical machinery. 
> Use proper electrical isolation, fusing, and ensure your motor controller is designed to be controlled via relay contacts. 
> Test with the motor disconnected first.


---

## Hardware requirements

- **Controllino Maxi**
- Garage door motor suitable for relay control:
  - One relay for **power/enable**
  - One relay for **direction**
- Optional sensors:
  - **Closed endstop** (recommended / effectively required for “closed” certainty)
  - **Open endstop**
  - **Light curtain / obstruction input**

## Pin mapping

### Inputs
- Local toggle button: `CONTROLLINO_A9`
- Closed endstop: `CONTROLLINO_A8` (**required for closed binary sensor**)
- Open endstop: `-1` (disabled by default)
- Light curtain: `-1` (disabled by default)

### Outputs
- Power relay: `CONTROLLINO_RELAY_08`
- Direction relay: `CONTROLLINO_RELAY_09`
- Status light: `CONTROLLINO_D11`
- Locked indicator: `CONTROLLINO_D10`

### Polarity
Configured with:
- `CLOSED_ENDSTOP_ACTIVE_LEVEL` (default `LOW`)
- `OPEN_ENDSTOP_ACTIVE_LEVEL` (default `LOW`)
- `LIGHT_CURTAIN_ACTIVE_LEVEL` (default `LOW`)

Adjust these to match your setup (NO/NC, pullups, etc.).