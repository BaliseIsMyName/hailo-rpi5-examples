#!/bin/bash
# Wrapper script to set up environment and run the camera script

# Sourcing the setup_env.sh script
source /home/raspi/hailo-rpi5-examples/setup_env.sh

# Export DISPLAY and XAUTHORITY for GUI access
export DISPLAY=:0
export XAUTHORITY=/home/raspi/.Xauthority

# Log les variables d'environnement (optionnel pour le débogage)
env | tee /home/raspi/wetthecatapp_env.log

# Log le lancement du script (optionnel pour le débogage)
echo "Running detection.py at $(date)" | tee -a /home/raspi/wetthecatapp.log

# Exécuter le script Python et rediriger la sortie et les erreurs
python -u /home/raspi/hailo-rpi5-examples/basic_pipelines/detection.py -i /dev/video0 2>&1 | tee -a /home/raspi/wetthecatapp.log

