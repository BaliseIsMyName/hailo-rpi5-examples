#!/bin/bash
# Wrapper script to set up environment and run the camera script

# Sourcing the setup_env.sh script
source /home/raspi/hailo-rpi5-examples/setup_env.sh

# Export DISPLAY and XAUTHORITY for GUI access
export DISPLAY=:0
export XAUTHORITY=/home/raspi/.Xauthority

# Log les variables d'environnement (optionnel pour le débogage)
env > /home/raspi/wetthecatapp_env.log

# Log le lancement du script (optionnel pour le débogage)
echo "Running instance_segmentation_test_camera.py at $(date)" >> /home/raspi/wetthecatapp.log

# Exécuter le script Python et rediriger la sortie et les erreurs
python /home/raspi/hailo-rpi5-examples/basic_pipelines/instance_segmentation_test_camera.py -i /dev/video0 >> /home/raspi/wetthecatapp.log 2>&1

