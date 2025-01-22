import gpiod
import time

# Définition des numéros de lignes pour les LED et le bouton
LED_PINS = {'rouge': 24, 'vert': 27, 'bleu': 22}
BUTTON_PIN = 23

# Nom du chip GPIO (à vérifier selon votre configuration)
CHIP_NAME = 'gpiochip0'

# Initialisation du chip GPIO
chip = gpiod.Chip(CHIP_NAME)

# Obtention des lignes correspondant aux LED
led_lines = {color: chip.get_line(pin) for color, pin in LED_PINS.items()}

# Obtention de la ligne correspondant au bouton
button_line = chip.get_line(BUTTON_PIN)

# Configuration des LED en sortie avec une valeur initiale à 1 (éteint)
for line in led_lines.values():
    line.request(consumer='led', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])

# Configuration du bouton en entrée avec détection de front descendant et pull-up interne
button_line.request(consumer='button', type=gpiod.LINE_REQ_EV_FALLING_EDGE, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)

# Liste des couleurs disponibles
couleurs = ['rouge', 'vert', 'bleu']
index_couleur = 0

print("Appuyez sur le bouton pour changer la couleur de la LED (Ctrl+C pour quitter).")

try:
    while True:
        # Attente d'un événement d'appui (front descendant) avec un timeout de 1 seconde
        event = button_line.event_wait(sec=1)
        if event:
            evt = button_line.event_read()
            # Selon la version de gpiod, l'accès au type d'événement peut différer
            try:
                event_type = evt.event_type
            except AttributeError:
                event_type = evt.type

            if event_type == gpiod.LineEvent.FALLING_EDGE:
                # Éteindre toutes les LED
                for line in led_lines.values():
                    line.set_value(1)
                # Allumer la LED de la couleur actuelle
                couleur_actuelle = couleurs[index_couleur]
                led_lines[couleur_actuelle].set_value(0)
                print(f"Couleur actuelle : {couleur_actuelle}")
                # Passer à la couleur suivante
                index_couleur = (index_couleur + 1) % len(couleurs)
                # Désactiver temporairement la détection des événements pour gérer les rebonds
                time.sleep(0.6)  # Délai de 600 ms

except KeyboardInterrupt:
    print("Interruption du programme.")

finally:
    # Éteindre toutes les LED et libérer les ressources
    for line in led_lines.values():
        line.set_value(1)
        line.release()
    button_line.release()
    chip.close()
