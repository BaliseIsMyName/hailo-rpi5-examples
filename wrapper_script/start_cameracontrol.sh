#!/bin/bash
# Wrapper script to set up environment and run server

# Sourcing the setup_env.sh script
source /home/raspi/hailo-rpi5-examples/setup_env.sh 

# Log les variables d'environnement (optionnel pour le débogage)
env | tee /home/raspi/camera_control_server_env.log

# Log le lancement du script (optionnel pour le débogage)
echo "Running camera_control_server.py at $(date)" | tee -a /home/raspi/cameraserver.log

# Exécuter le script Python et rediriger la sortie et les erreurs
exec python -u /home/raspi/hailo-rpi5-examples/basic_pipelines/camera_control_server.py 2>&1 | tee -a /home/raspi/camera_control_server.log