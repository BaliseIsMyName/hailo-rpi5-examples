#!/bin/bash
# Wrapper script to set up environment and run the camera script

# Sourcing the setup_env.sh script
source /home/raspi/hailo-rpi5-examples/setup_env.sh

# Export DISPLAY and XAUTHORITY for GUI access
export DISPLAY=:0
export XAUTHORITY=/home/raspi/.Xauthority

# Log le lancement du script (optionnel pour le débogage)
echo "Running detection.py at $(date)" >> /home/raspi/wetthecatapp.log

# Exécuter le script Python en remplaçant le shell
exec python -u /home/raspi/hailo-rpi5-examples/basic_pipelines/detection.py -i /dev/video0


