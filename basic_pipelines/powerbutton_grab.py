#!/usr/bin/env python3
import asyncio
import subprocess
from evdev import InputDevice, categorize, ecodes
import gpiod

# Chemin de l'appareil pour le bouton Power
POWER_DEVICE = "/dev/input/event5"

# Numéros de lignes pour les LED (adapter selon votre câblage)
LED_BLUE_PIN   = 16  # bleu
LED_GREEN_PIN  = 5   # vert
LED_RED_PIN    = 6   # rouge

CHIP_NAME = 'gpiochip0'  # Nom du chip GPIO (souvent "gpiochip0" sur Raspberry Pi)

# Modes d'affichage :
# mode 0 : LED statique verte
# mode 1 : clignotement rouge/bleu (plus rapide)
# mode 2 : clignotement rouge rapide durant la séquence d'arrêt, puis LED rouge statique puis extinction
mode = 0

# Référence aux tâches de clignotement (pour pouvoir les annuler)
blinking_task = None
red_blink_task = None

# Initialisation du chip et des lignes LED (elles seront demandées plus tard)
chip = gpiod.Chip(CHIP_NAME)
led_blue  = chip.get_line(LED_BLUE_PIN)
led_green = chip.get_line(LED_GREEN_PIN)
led_red   = chip.get_line(LED_RED_PIN)


def setup_leds():
    """
    Configure les trois lignes LED en sorties avec la valeur initiale à 1 (LED éteinte).
    On part du principe d'un montage en actif bas (0 = LED allumée).
    """
    for name, line in (("blue", led_blue), ("green", led_green), ("red", led_red)):
        line.request(consumer=f"led_{name}", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])


def set_led_state(blue, green, red):
    """
    Définit l'état des LED.
    Les paramètres doivent être 0 (allumé) ou 1 (éteint) pour chaque couleur.
    """
    led_blue.set_value(blue)
    led_green.set_value(green)
    led_red.set_value(red)


async def blinking():
    """
    Coroutine du clignotement en mode 1 : 
    Alterne entre rouge et bleu avec une fréquence de 0,25 seconde.
    La LED verte reste éteinte.
    """
    global mode
    toggle = False
    try:
        while mode == 1:
            if toggle:
                # Allume rouge, éteint bleu
                set_led_state(blue=1, green=1, red=0)
            else:
                # Allume bleu, éteint rouge
                set_led_state(blue=0, green=1, red=1)
            toggle = not toggle
            await asyncio.sleep(0.25)
    except asyncio.CancelledError:
        return


async def blinking_red_rapide():
    """
    Coroutine du clignotement rouge rapide en mode 2.
    Clignote en rouge (allumé/éteint) avec une fréquence de 0,2 seconde.
    """
    try:
        while True:
            # Rouge allumé
            set_led_state(blue=1, green=1, red=0)
            await asyncio.sleep(0.2)
            # Toutes éteintes
            set_led_state(blue=1, green=1, red=1)
            await asyncio.sleep(0.2)
    except asyncio.CancelledError:
        return


def services_stopped(services):
    """
    Vérifie si tous les services de la liste sont arrêtés (inactifs).
    Renvoie True si tous sont inactifs, False sinon.
    """
    for svc in services:
        try:
            result = subprocess.run(
                ["systemctl", "is-active", svc],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True)
            if result.stdout.strip() != "inactive":
                return False
        except Exception as e:
            print(f"Erreur lors de la vérification du service {svc}: {e}")
            return False
    return True


async def shutdown_sequence():
    """
    Lance la séquence d'arrêt :
      - Stoppe les services et démarre un clignotement rouge rapide (mode 2) pendant l'arrêt.
      - Attends que les services spécifiés soient arrêtés.
      - Une fois arrêtés, fixe la LED en rouge statique.
      - Après un délai d'affichage, déclenche l'arrêt du Raspberry Pi.
    """
    services = ["wetthecat.service", "uvicorn.service"]
    print("Mode 2 activé : lancement de la séquence d'arrêt.")

    # Lancer le clignotement rouge rapide
    global red_blink_task
    red_blink_task = asyncio.create_task(blinking_red_rapide())

    # Tenter d'arrêter explicitement les services
    subprocess.run(["sudo", "systemctl", "stop", "wetthecat.service"])
    subprocess.run(["sudo", "systemctl", "stop", "uvicorn.service"])

    print("Attente de l'arrêt des services...")
    while not services_stopped(services):
        await asyncio.sleep(50)

    # On peut ajouter un délai supplémentaire pour s'assurer que les routines de fin ont le temps de s'exécuter
    await asyncio.sleep(15)

    # Annuler le clignotement rouge rapide
    red_blink_task.cancel()
    try:
        await red_blink_task
    except asyncio.CancelledError:
        pass

    # Afficher la LED rouge en continu
    set_led_state(blue=1, green=1, red=0)
    print("Services arrêtés, LED rouge statique.")
    
    await asyncio.sleep(2)
    print("Extinction du Raspberry Pi...")
    subprocess.run(["sudo", "systemctl", "poweroff"])


async def wait_for_keypress_with_timeout(dev, timeout):
    """
    Attend une pression sur le bouton KEY_POWER dans dev pendant 'timeout' secondes.
    Renvoie True si l'appui est détecté, False sinon.
    """
    try:
        # Utilisation de wait_for pour attendre un événement dans le délai imparti
        return await asyncio.wait_for(asyncio.create_task(
            wait_for_keypress(dev)), timeout=timeout)
    except asyncio.TimeoutError:
        return False


async def wait_for_keypress(dev):
    """
    Attend la première pression sur le bouton KEY_POWER et renvoie True.
    """
    async for event in dev.async_read_loop():
        if event.type == ecodes.EV_KEY:
            key_event = categorize(event)
            if key_event.keycode == "KEY_POWER" and key_event.keystate == key_event.key_down:
                return True


async def handle_events(dev):
    """
    Gère les événements du bouton Power et l'évolution des modes.
      - Mode 0 : LED verte statique.
      - Premier appui → Passage en mode 1 (clignotement rouge/bleu).
        Ensuite, si aucune nouvelle pression n'est détectée dans les 10 secondes,
        le système revient en mode 0 (vert statique).
      - Si, en mode 1, un deuxième appui intervient dans les 10 sec, passage en mode 2,
        déclenchant la séquence d'arrêt.
    """
    global mode, blinking_task

    while True:
        # Attente d'un appui en mode 0
        event = await wait_for_keypress(dev)
        if event:
            if mode == 0:
                # Passage en mode 1 : clignotement rouge/bleu
                mode = 1
                blinking_task = asyncio.create_task(blinking())
                print("Mode 1 : clignotement rouge/bleu")
                # En mode 1, on attend un deuxième appui pendant 10 secondes
                second_press = await wait_for_keypress_with_timeout(dev, 10)
                if second_press:
                    # Passage en mode 2 : exécution de la séquence d'arrêt
                    mode = 2
                    if blinking_task is not None:
                        blinking_task.cancel()
                        try:
                            await blinking_task
                        except asyncio.CancelledError:
                            pass
                    await shutdown_sequence()
                    # Après shutdown_sequence, on arrête la boucle (le système s'éteindra)
                    break
                else:
                    # Timeout : aucun deuxième appui dans les 10 sec, retour au mode initial (vert)
                    print("Aucun deuxième appui détecté, retour au mode initial.")
                    if blinking_task is not None:
                        blinking_task.cancel()
                        try:
                            await blinking_task
                        except asyncio.CancelledError:
                            pass
                    mode = 0
                    # Fixer la LED en vert
                    set_led_state(blue=1, green=0, red=1)
                    print("Mode 0 : LED verte statique")
            # On ignore les appuis dans les autres modes pour cette boucle
            

async def main():
    # Ouvrir l'appareil input pour le bouton POWER
    dev = InputDevice(POWER_DEVICE)
    dev.grab()

    # Configurer les LED
    setup_leds()

    # Mode initial (0) : LED verte statique
    global mode
    mode = 0
    set_led_state(blue=1, green=0, red=1)  # seule la LED verte allumée (0 = allumé)
    print("Script démarré.")
    print("Mode initial (0) : LED verte statique")
    print("Appuyez sur le bouton Power pour changer le mode.")

    try:
        await handle_events(dev)
    finally:
        # À la sortie, éteindre les LED et libérer les ressources
        set_led_state(blue=1, green=1, red=1)
        led_blue.release()
        led_green.release()
        led_red.release()
        chip.close()
        dev.ungrab()

if __name__ == "__main__":
    asyncio.run(main())
