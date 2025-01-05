#!/bin/bash
# Wrapper script to set up environment and run server
#xterm -geometry '180x100+0+720'
# Sourcing the setup_env.sh script
source /home/raspi/hailo-rpi5-examples/setup_env.sh

# Export DISPLAY and XAUTHORITY for GUI access
export DISPLAY=:0
export XAUTHORITY=/home/raspi/.Xauthority

# Log les variables d'environnement (optionnel pour le débogage)
env | tee /home/raspi/server_env.log

# Log le lancement du script (optionnel pour le débogage)
echo "Running Uvicorn Server App.py at $(date)" | tee /home/raspi/server.log

uvicorn basic_pipelines.app.main:app --host 0.0.0.0 --port 8000 2>&1 | tee -a /home/raspi/server.log


