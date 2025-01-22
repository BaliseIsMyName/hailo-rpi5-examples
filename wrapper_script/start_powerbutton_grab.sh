#!/bin/bash
# Wrapper script to set up environment and run the camera script

# Sourcing the setup_env.sh script
source /home/raspi/hailo-rpi5-examples/setup_env.sh


# Log le lancement du script (optionnel pour le débogage)
echo "Running powerbutton_grab.py at $(date)" >> /home/raspi/powerbutton_grab.log

# Exécuter le script Python en remplaçant le shell
exec python -u /home/raspi/hailo-rpi5-examples/basic_pipelines/powerbutton_grab.py


