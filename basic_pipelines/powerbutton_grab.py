#!/usr/bin/env python3
import asyncio
import subprocess
import time
import os
import yaml

from evdev import InputDevice, categorize, ecodes
import gpiod

# ========== PARTIE 1 : Configuration générale ==========

# -- Fichier config.yaml (pour le mode) --
CONFIG_PATH = "/home/raspi/hailo-rpi5-examples/basic_pipelines/config.yaml"  # Adapter le chemin

def load_mode() -> int:
    """Lit la valeur 'mode' dans config.yaml, retourne 0 si inexistant."""
    if not os.path.exists(CONFIG_PATH):
        return 0
    with open(CONFIG_PATH, "r") as f:
        config = yaml.safe_load(f) or {}
    camera_movement = config.get("camera_movement", {})
    return camera_movement.get("mode", 0)

def save_mode(mode: int) -> None:
    """Écrit la valeur 'mode' dans config.yaml sous camera_movement['mode']."""  
    config = {}
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, "r") as f:
            config = yaml.safe_load(f) or {}

    # S'assurer que la section camera_movement existe
    if "camera_movement" not in config:
        config["camera_movement"] = {}

    # Mettre à jour le mode
    config["camera_movement"]["mode"] = mode

    # Réécriture complète du fichier config.yaml
    with open(CONFIG_PATH, "w") as f:
        yaml.safe_dump(
            config,
            f,
            sort_keys=False,         # On garde l'ordre des clés
            default_flow_style=False # Écriture “multiligne” au lieu d'une seule ligne
        )



# ========== PARTIE 2 : Gestion du bouton Power (/dev/input/event5) et LEDS 16,5,6 ==========

POWER_DEVICE = "/dev/input/event5"

LED_BLUE_PIN_OLD   = 16  # bleu
LED_GREEN_PIN_OLD  = 5   # vert
LED_RED_PIN_OLD    = 6   # rouge

CHIP_NAME = 'gpiochip0'  # Contrôleur GPIO

# Variable globale 'mode' pour la logique power
mode_power = 0

# Référence aux tâches de clignotement
blinking_task = None
red_blink_task = None

chip = gpiod.Chip(CHIP_NAME)
led_blue_old  = chip.get_line(LED_BLUE_PIN_OLD)
led_green_old = chip.get_line(LED_GREEN_PIN_OLD)
led_red_old   = chip.get_line(LED_RED_PIN_OLD)

def setup_old_leds():
    """Configure en sortie actif bas les 3 LED de la logique power."""
    for name, line in (("blue", led_blue_old), ("green", led_green_old), ("red", led_red_old)):
        line.request(consumer=f"led_{name}", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])

def set_old_led_state(blue, green, red):
    """0 = allumé, 1 = éteint."""
    led_blue_old.set_value(blue)
    led_green_old.set_value(green)
    led_red_old.set_value(red)

async def blinking():
    """Clignotement rouge/bleu pour mode 1 (power)."""
    global mode_power
    toggle = False
    try:
        while mode_power == 1:
            if toggle:
                # Allume rouge
                set_old_led_state(blue=1, green=1, red=0)
            else:
                # Allume bleu
                set_old_led_state(blue=0, green=1, red=1)
            toggle = not toggle
            await asyncio.sleep(0.1)
    except asyncio.CancelledError:
        return

async def blinking_red_rapide():
    """Clignotement rouge rapide en mode 2 (power)."""
    try:
        while True:
            set_old_led_state(blue=1, green=1, red=0)
            await asyncio.sleep(0.1)
            set_old_led_state(blue=1, green=1, red=1)
            await asyncio.sleep(0.1)
    except asyncio.CancelledError:
        return

def services_stopped(services):
    """Vérifie si tous les services sont 'inactive'."""
    for svc in services:
        try:
            result = subprocess.run(["systemctl", "is-active", svc],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE,
                                    text=True)
            if result.stdout.strip() != "inactive":
                return False
        except Exception as e:
            print(f"Erreur check {svc}: {e}")
            return False
    return True

async def shutdown_sequence():
    """Séquence d'arrêt (mode 2)."""
    services = ["wetthecat.service", "uvicorn.service"]
    print("Mode 2 activé : lancement de l'arrêt (power).")

    global red_blink_task
    red_blink_task = asyncio.create_task(blinking_red_rapide())

    # Arrêt user services
    subprocess.run([
        "sudo","-u","raspi",
        "env","XDG_RUNTIME_DIR=/run/user/1000",
        "systemctl","--user","stop","wetthecat.service"
    ])
    subprocess.run([
        "sudo","-u","raspi",
        "env","XDG_RUNTIME_DIR=/run/user/1000",
        "systemctl","--user","stop","uvicorn.service"
    ])

    print("Attente arrêt services...")
    while not services_stopped(services):
        await asyncio.sleep(15)

    # Délai supp
    await asyncio.sleep(10)

    # Annule clignotement
    red_blink_task.cancel()
    try:
        await red_blink_task
    except asyncio.CancelledError:
        pass

    # LED rouge statique
    set_old_led_state(blue=1, green=1, red=0)
    print("Services arrêtés, LED rouge (power) statique.")
    await asyncio.sleep(2)
    print("Extinction Raspberry Pi...")
    subprocess.run(["sudo","systemctl","poweroff"])

async def wait_for_keypress_with_timeout(dev, timeout):
    try:
        return await asyncio.wait_for(asyncio.create_task(wait_for_keypress(dev)), timeout=timeout)
    except asyncio.TimeoutError:
        return False

async def wait_for_keypress(dev):
    async for event in dev.async_read_loop():
        if event.type == ecodes.EV_KEY:
            key_event = categorize(event)
            if key_event.keycode == "KEY_POWER" and key_event.keystate == key_event.key_down:
                return True

async def handle_events_power(dev):
    """
    Gère l'appui sur /dev/input/event5 (bouton power) → modes 0->1->2 (extinction).
    """
    global mode_power, blinking_task
    while True:
        # Attente d'un appui en mode 0
        event = await wait_for_keypress(dev)
        if event:
            if mode_power == 0:
                mode_power = 1
                blinking_task = asyncio.create_task(blinking())
                print("Power Mode 1 : clignotement rouge/bleu")
                second_press = await wait_for_keypress_with_timeout(dev, 10)
                if second_press:
                    mode_power = 2
                    if blinking_task:
                        blinking_task.cancel()
                        try:
                            await blinking_task
                        except asyncio.CancelledError:
                            pass
                    await shutdown_sequence()
                    break
                else:
                    print("Pas de 2e appui, retour mode 0 (power).")
                    if blinking_task:
                        blinking_task.cancel()
                        try:
                            await blinking_task
                        except asyncio.CancelledError:
                            pass
                    mode_power = 0
                    set_old_led_state(blue=1, green=0, red=1)  # LED verte
                    print("Power Mode 0 : LED verte statique")


# ========== PARTIE 3 : Bouton GPIO23 + LEDS 24,27,22 pour config.yaml ==========

# Broches pour le "2e" jeu de LED
LED_RED_PIN   = 24
LED_GREEN_PIN = 27
LED_BLUE_PIN  = 22

# Bouton sur GPIO23
BUTTON_PIN = 23

# Récupération des lignes
led_line_red   = chip.get_line(LED_RED_PIN)
led_line_green = chip.get_line(LED_GREEN_PIN)
led_line_blue  = chip.get_line(LED_BLUE_PIN)
btn_line       = chip.get_line(BUTTON_PIN)

def setup_new_leds_and_button():
    """Configure le bouton (entrée) et les 3 LED (sortie)."""
    # LEDs en sortie, actif bas => 1 = éteint
    led_line_red.request(
        consumer="led_r",
        type=gpiod.LINE_REQ_DIR_OUT,
        default_vals=[1]
    )
    led_line_green.request(
        consumer="led_g",
        type=gpiod.LINE_REQ_DIR_OUT,
        default_vals=[1]
    )
    led_line_blue.request(
        consumer="led_b",
        type=gpiod.LINE_REQ_DIR_OUT,
        default_vals=[1]
    )
    # Bouton en entrée + détection d'événements
    btn_line.request(
        consumer="btn_cfg",
        type=gpiod.LINE_REQ_EV_BOTH_EDGES,
        default_vals=[1],
        flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP
    )

def set_config_led(mode):
    """
    0 => LED rouge
    1 => LED verte
    2 => LED bleue
    On part du principe 0 = allumé, 1 = éteint (actif bas).
    """
    if mode == 0:
        led_line_red.set_value(0)
        led_line_green.set_value(1)
        led_line_blue.set_value(1)
    elif mode == 1:
        led_line_red.set_value(1)
        led_line_green.set_value(0)
        led_line_blue.set_value(1)
    else:
        led_line_red.set_value(1)
        led_line_green.set_value(1)
        led_line_blue.set_value(0)

async def handle_button_gpio23():
    """
    Boucle asynchrone surveillant la ligne GPIO23.
    À chaque front descendant => on incrémente le mode (0→1→2→0) dans config.yaml,
    puis on met à jour la LED correspondante.
    """
    # Lecture du mode initial
    current_mode = load_mode()
    set_config_led(current_mode)
    print(f"[GPIO23] Mode config.yaml initial = {current_mode}")

    while True:
        # On attend un événement
        ev_line = btn_line.event_wait()
        if ev_line:
            event = btn_line.event_read()
            # On détecte un front descendant
            if event.type == gpiod.LineEvent.FALLING_EDGE:
                # On incrémente
                current_mode = (current_mode + 1) % 3
                # On l'enregistre
                save_mode(current_mode)
                print(f"[GPIO23] Nouveau mode={current_mode}, sauvegardé dans config.yaml.")
                # Mise à jour LED
                set_config_led(current_mode)
                
        # Vérifie si le fichier config.yaml a changé la valeur du mode
        new_mode = load_mode()
        if new_mode != current_mode:
            current_mode = new_mode
            set_config_led(current_mode)
            print(f"[GPIO23] Mode détecté à {current_mode} (changement manuel dans config.yaml)")

        await asyncio.sleep(0.01)


# ========== PARTIE 4 : main() global ==========

async def main():
    # 1) Ouvrir l'appareil input pour /dev/input/event5 (power)
    power_dev = InputDevice(POWER_DEVICE)
    power_dev.grab()

    # 2) Préparer les LED "power"
    setup_old_leds()
    # Mode initial = 0 => LED verte
    global mode_power
    mode_power = 0
    set_old_led_state(blue=1, green=0, red=1)
    print("Script démarré (power). Mode 0 : LED verte statique")

    # 3) Préparer le bouton GPIO23 + LED (24,27,22) pour config.yaml
    setup_new_leds_and_button()

    # 4) Lancer les deux tâches en parallèle
    try:
        task_power = asyncio.create_task(handle_events_power(power_dev))
        task_gpio23 = asyncio.create_task(handle_button_gpio23())

        await asyncio.gather(task_power, task_gpio23)

    finally:
        # À la sortie
        # Libérer LED power
        set_old_led_state(blue=1, green=1, red=1)
        led_blue_old.release()
        led_green_old.release()
        led_red_old.release()

        # Libérer LED config
        led_line_red.set_value(1)
        led_line_green.set_value(1)
        led_line_blue.set_value(1)
        led_line_red.release()
        led_line_green.release()
        led_line_blue.release()

        # Libérer bouton /dev/input
        power_dev.ungrab()

        # Libérer bouton gpio23
        btn_line.release()

        chip.close()
        print("Terminé.")

if __name__ == "__main__":
    import sys
    from evdev import InputDevice, categorize, ecodes
    asyncio.run(main())
