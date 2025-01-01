#!/bin/bash
# Wrapper script to set up environment and run server

# Sourcing the setup_env.sh script
source /home/raspi/hailo-rpi5-examples/setup_env.sh &&
uvicorn basic_pipelines.app.main:app --host 0.0.0.0 --port 8000 2>&1 | tee -a /home/raspi/server.log
# Log les variables d'environnement (optionnel pour le débogage)
env | tee /home/raspi/server_env.log

# Log le lancement du script (optionnel pour le débogage)
echo "Running instance_segmentation_test_camera.py at $(date)" | tee /home/raspi/server.log

# Exécuter le script Python et rediriger la sortie et les erreurs
