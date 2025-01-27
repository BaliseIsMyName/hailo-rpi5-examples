# /home/raspi/hailo-rpi5-examples/basic_pipelines/app/main.py

from fastapi import FastAPI, Depends, HTTPException, status, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.security import OAuth2PasswordRequestForm
from fastapi.responses import HTMLResponse, RedirectResponse, JSONResponse, FileResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles  # Importer StaticFiles
from sqlalchemy.orm import Session
from fastapi.middleware.cors import CORSMiddleware
from datetime import timedelta
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from starlette.requests import Request
from typing import List
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from urllib.parse import unquote

from . import models, schemas, crud, utils, database, oauth2
from .config import ADMIN_PASSWORD, STREAM_URL  # Importer STREAM_URL

import yaml
import shutil
import os
import asyncio

# Emplacement du fichier config.yaml
CONFIG_PATH = "/home/raspi/hailo-rpi5-examples/basic_pipelines/config.yaml"

class ConfigEventHandler(FileSystemEventHandler):
    def __init__(self, app: FastAPI, config_path: str):
        super().__init__()
        self.app = app
        self.config_path = os.path.abspath(config_path)
        self.previous_config = read_config()

    def on_modified(self, event):
        if os.path.abspath(event.src_path) == self.config_path:
            print(f"{self.config_path} a été modifié.")
            try:
                # Lire la nouvelle configuration
                new_config = read_config()
                updates = {}

                # Détecter les changements dans 'camera_movement' -> 'mode'
                old_mode = self.previous_config.get("camera_movement", {}).get("mode", 0)
                new_mode = new_config.get("camera_movement", {}).get("mode", 0)
                if old_mode != new_mode:
                    updates["mode"] = new_mode

                # Détecter les changements dans 'track_objects'
                old_track_objects = self.previous_config.get("track_objects", [])
                new_track_objects = new_config.get("track_objects", [])
                if old_track_objects != new_track_objects:
                    updates["track_objects"] = new_track_objects

                # Détecter les changements dans 'track_objects_list'
                old_track_objects_list = self.previous_config.get("track_objects_list", [])
                new_track_objects_list = new_config.get("track_objects_list", [])
                if old_track_objects_list != new_track_objects_list:
                    updates["track_objects_list"] = new_track_objects_list

                # Mettre à jour l'état précédent
                self.previous_config = new_config

                # Diffuser les mises à jour via WebSocket
                for update_type, data in updates.items():
                    asyncio.run_coroutine_threadsafe(
                        self.app.state.manager.broadcast_update(update_type, data),
                        self.app.state.loop
                    )
            except Exception as e:
                print(f"Erreur lors de la gestion de la modification de config.yaml: {e}")


def read_config():
    """Lit le contenu de config.yaml et retourne un dictionnaire Python."""
    with open(CONFIG_PATH, "r") as f:
        data = yaml.safe_load(f)
        if data is None:
            print("config.yaml est vide ou invalide. Retour d'un dictionnaire vide.")
            return {}
        return data


def write_config(config_data):
    """Écrit le dictionnaire Python dans config.yaml."""
    with open(CONFIG_PATH, "w") as f:
        yaml.safe_dump(config_data, f)

def archive_video(filename: str, from_folder="/home/raspi/Videos/records", archive_folder="/home/raspi/Videos/archives"):
    """
    Déplace le fichier 'filename' de from_folder vers archive_folder.
    """
    filename = unquote(filename)  # Décoder les caractères spéciaux
    src = os.path.join(from_folder, filename)
    dst = os.path.join(archive_folder, filename)
    
    print(f"Tentative de déplacement : {src} -> {dst}")  # Ajout pour debug
    
    if not os.path.exists(src):
        print(f"Fichier introuvable : {src}")
        return False
    
    # Créer le dossier d'archives s'il n'existe pas
    os.makedirs(archive_folder, exist_ok=True)

    try:
        shutil.move(src, dst)
        print(f"Déplacement réussi : {src} -> {dst}")
        return True
    except Exception as e:
        print(f"Erreur d'archivage de {src} vers {dst}: {e}")
        return False


# Configurer le limiteur
limiter = Limiter(key_func=get_remote_address)

app = FastAPI()

app.state.limiter = limiter
app.add_exception_handler(429, _rate_limit_exceeded_handler)

# Monter le répertoire static pour les fichiers CSS, JS, etc.
app.mount("/static", StaticFiles(directory="/home/raspi/hailo-rpi5-examples/basic_pipelines/app/static"), name="static")

app.mount("/records_static", StaticFiles(directory="/home/raspi/Videos/records"), name="records_static")

# Configurer les templates
templates = Jinja2Templates(directory="/home/raspi/hailo-rpi5-examples/basic_pipelines/app/templates")

# Permettre les CORS si nécessaire
app.add_middleware(
      CORSMiddleware,
      allow_origins=["*"],  # Modifier en production
      allow_credentials=True,
      allow_methods=["*"],
      allow_headers=["*"],
)

# Créer les tables si elles n'existent pas
models.Base.metadata.create_all(bind=database.engine)

@app.on_event("startup")
async def startup_event():
    # Associer le gestionnaire de connexions au state de l'app
    app.state.manager = manager
    app.state.loop = asyncio.get_event_loop()

    # Configurer Watchdog
    event_handler = ConfigEventHandler(app, CONFIG_PATH)
    observer = Observer()
    observer.schedule(event_handler, path=os.path.dirname(CONFIG_PATH), recursive=False)
    observer.start()
    app.state.observer = observer
    print("Watchdog démarré pour surveiller config.yaml.")

@app.on_event("shutdown")
async def shutdown_event():
    observer: Observer = app.state.observer # type: ignore
    observer.stop()
    observer.join()
    print("Watchdog arrêté.")


# Gestionnaire d'exceptions pour rediriger les 401 vers /login
@app.exception_handler(StarletteHTTPException)
async def http_exception_handler(request: Request, exc: StarletteHTTPException):
      if exc.status_code == status.HTTP_401_UNAUTHORIZED:
            return RedirectResponse(url="/loginN", status_code=status.HTTP_302_FOUND)
      return RedirectResponse(url="/error", status_code=exc.status_code)  # Optionnel: gérer d'autres erreurs

# Gestionnaire de WebSockets
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket):
        async with self.lock:
            self.active_connections.append(websocket)
        print(f"Client connecté: {websocket.client}")

    async def disconnect(self, websocket: WebSocket):
        async with self.lock:
            if websocket in self.active_connections:
                self.active_connections.remove(websocket)
                print(f"Client déconnecté: {websocket.client}")

    async def broadcast_update(self, update_type: str, data: dict):
        """
        Diffuse une mise à jour à tous les clients connectés.

        :param update_type: Type de mise à jour ('mode', 'track_objects', 'track_objects_list')
        :param data: Données associées à la mise à jour
        """
        message = {"type": update_type, "data": data}
        async with self.lock:
            for connection in self.active_connections:
                try:
                    await connection.send_json(message)
                    print(f"Message '{update_type}' envoyé au client {connection.client}: {message}")
                except Exception as e:
                    print(f"Erreur lors de l'envoi du message '{update_type}' au client {connection.client}: {e}")
                    await self.disconnect(connection)

# Instancier le gestionnaire
manager = ConnectionManager()

# Route WebSocket pour le mode avec authentification via cookies
@app.websocket("/ws/mode")
async def websocket_mode_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("WebSocket accepté.")

    # Récupérer le token depuis les cookies
    token = websocket.cookies.get("access_token")
    if not token:
        print("Token manquant dans les cookies.")
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
        return

    # Supprimer le préfixe "Bearer " si présent
    if token.startswith("Bearer "):
        token = token[7:]

    try:
        # Décoder le token pour obtenir le nom d'utilisateur
        username = utils.decode_access_token(token)
        if username is None:
            print("Token invalide ou expiré.")
            await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
            return

        # Vérifier si l'utilisateur existe dans la base de données
        db: Session = database.SessionLocal()
        user = crud.get_user_by_username(db, username=username)
        db.close()
        if user is None:
            print("Utilisateur non trouvé.")
            await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
            return

        print(f"Utilisateur authentifié: {username}")

    except Exception as e:
        print(f"Erreur d'authentification WebSocket: {e}")
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
        return

    # Ajouter la connexion au gestionnaire
    await manager.connect(websocket)
    try:
        while True:
            try:
                # Recevoir des messages du client (si nécessaire)
                data = await websocket.receive_text()
                print(f"Message reçu du client {websocket.client}: {data}")
                # Vous pouvez traiter les messages ici si nécessaire
            except WebSocketDisconnect as e:
                print(f"WebSocket déconnecté par le client {websocket.client}: {e}")
                await manager.disconnect(websocket)
                break
            except Exception as e:
                print(f"Erreur WebSocket avec le client {websocket.client}: {e}")
                await manager.disconnect(websocket)
                break

    except WebSocketDisconnect as e:
        print(f"WebSocket déconnecté par le client {websocket.client}: {e}")
        await manager.disconnect(websocket)
    except Exception as e:
        print(f"Erreur WebSocket avec le client {websocket.client}: {e}")
        await manager.disconnect(websocket)


# Page d'accueil (login requis)
@app.get("/", response_class=HTMLResponse)
async def read_home(request: Request, current_user: schemas.User = Depends(oauth2.get_current_user)):
    folder = "/home/raspi/Videos/records"
    archive_folder = "/home/raspi/Videos/archives"
    videos = []
    archives = []
    
    if os.path.exists(folder):
        files = [f for f in os.listdir(folder) if f.endswith(".mp4")]
        # On trie par ctime desc
        files_info = []
        for fname in files:
            fullpath = os.path.join(folder, fname)
            st = os.stat(fullpath)
            files_info.append({
                "filename": fname,
                "ctime": st.st_ctime
            })
        files_info.sort(key=lambda x: x["ctime"], reverse=True)
        # On prend les 10 premiers
        top10 = files_info[:10]
        videos = top10
    
    if os.path.exists(archive_folder):
        archive_files = [f for f in os.listdir(archive_folder) if f.endswith(".mp4")]
        archives = [{"filename": f} for f in archive_files]

    return templates.TemplateResponse("index.html", {
        "request": request,
        "user": current_user,
        "videos": videos,
        "archives": archives
    })

# Page de login
@app.get("/loginN", response_class=HTMLResponse)
async def get_login(request: Request):
      return templates.TemplateResponse("login.html", {"request": request, "user": None})

# Page d'inscription
@app.get("/register-page", response_class=HTMLResponse)
async def get_register(request: Request):
      return templates.TemplateResponse("register.html", {"request": request, "user": None})

# Endpoint d'inscription avec vérification du mot de passe admin
@app.post("/register", response_model=schemas.User)
def register(user: schemas.UserRegister, db: Session = Depends(oauth2.get_db)):
      # Vérifier le mot de passe admin
      if user.admin_password != ADMIN_PASSWORD:
            raise HTTPException(status_code=403, detail="Forbidden: Invalid admin password")

      # Vérifier si le username ou l'email existe déjà
      db_user = crud.get_user_by_username(db, username=user.username)
      if db_user:
            raise HTTPException(status_code=400, detail="Username already registered")
      db_user = crud.get_user_by_email(db, email=user.email)
      if db_user:
            raise HTTPException(status_code=400, detail="Email already registered")
      
      # Créer le nouvel utilisateur
      return crud.create_user(db=db, user=schemas.UserCreate(username=user.username, email=user.email, password=user.password))

# Endpoint de connexion (login)
@app.post("/token", response_model=schemas.Token)
def login_for_access_token(response: Response, form_data: OAuth2PasswordRequestForm = Depends(), db: Session = Depends(oauth2.get_db)):
      user = crud.get_user_by_username(db, username=form_data.username)
      if not user:
            user = crud.get_user_by_email(db, email=form_data.username)  # Permettre la connexion par email
      if not user:
            raise HTTPException(status_code=400, detail="Incorrect username or password")
      if not utils.verify_password(form_data.password, user.hashed_password):
            raise HTTPException(status_code=400, detail="Incorrect username or password")
      access_token_expires = timedelta(minutes=utils.ACCESS_TOKEN_EXPIRE_MINUTES)
      access_token = utils.create_access_token(
            data={"sub": user.username}, expires_delta=access_token_expires
      )
      # Définir le token dans un cookie sécurisé
      response.set_cookie(
            key="access_token",
            value=f"Bearer {access_token}",
            httponly=True,      # Accessible uniquement par le serveur
            secure=True,        # Transmis uniquement via HTTPS
            samesite='strict'   # Protection CSRF
      )
      return {"access_token": access_token, "token_type": "bearer"}

# Endpoint de déconnexion
@app.post("/logout")
def logout(response: Response):
      response.delete_cookie(key="access_token")
      return {"msg": "Successfully logged out"}

# Endpoint protégé pour obtenir les infos de l'utilisateur connecté
@app.get("/users/me", response_model=schemas.User)
def read_users_me(current_user: schemas.User = Depends(oauth2.get_current_user)):
      return current_user

# Page protégée (stream)
@app.get("/stream", response_class=HTMLResponse)
async def read_stream(request: Request, current_user: schemas.User = Depends(oauth2.get_current_user)):
      return templates.TemplateResponse("stream.html", {
            "request": request,
            "user": current_user,
            "stream_url": STREAM_URL  # Passer l'URL du flux au template
      })
      
# Page protégée (stream)
@app.get("/stream2", response_class=HTMLResponse)
async def read_stream(request: Request, current_user: schemas.User = Depends(oauth2.get_current_user)):
      return templates.TemplateResponse("stream2.html", {
            "request": request,
            "user": current_user
      })

@app.get("/profile", response_class=HTMLResponse)
async def get_profile(request: Request, current_user: schemas.User = Depends(oauth2.get_current_user)):
    return templates.TemplateResponse("profile.html", {"request": request, "user": current_user})

@app.get("/camera_mode")
def get_camera_mode(current_user: schemas.User = Depends(oauth2.get_current_user)):
    """
    Récupère le 'mode' actuel dans config.yaml (camera_movement -> mode).
    """
    config = read_config()
    current_mode = config.get("camera_movement", {}).get("mode", 0)
    return {"mode": current_mode}

# Modifier la route POST /camera_mode pour émettre une mise à jour via WebSocket
@app.post("/camera_mode")
async def set_camera_mode(
    data: dict,
    current_user: schemas.User = Depends(oauth2.get_current_user)
):
    """
    Met à jour le 'mode' dans config.yaml (camera_movement -> mode).
    data doit être un JSON contenant {"mode": <valeur_entière>}.
    """
    new_mode = data.get("mode")
    if new_mode is None:
        raise HTTPException(status_code=400, detail="No mode provided")

    # Valider que new_mode est un entier entre 0 et 2
    if not isinstance(new_mode, int) or new_mode not in [0, 1, 2]:
        raise HTTPException(status_code=400, detail="Invalid mode value. Must be 0, 1, or 2.")

    config = read_config()
    # On s'assure que camera_movement existe bien
    if "camera_movement" not in config:
        config["camera_movement"] = {}
    config["camera_movement"]["mode"] = new_mode

    write_config(config)

    # Émettre la mise à jour via WebSocket
    await manager.broadcast_update('mode', {'mode': new_mode})

    return {"mode": new_mode}

@app.get("/track_objects")
def get_track_objects(current_user: schemas.User = Depends(oauth2.get_current_user)):
    """
    Récupère la liste track_objects dans config.yaml
    (par défaut cat/person) et la renvoie sous forme JSON.
    """
    config = read_config()
    # On attend une liste (ex: ["cat","person"])
    objects_list = config.get("track_objects", [])
    # On s’assure que c’est une liste, sinon on renvoie au moins []
    if not isinstance(objects_list, list):
        objects_list = []
    return {"track_objects": objects_list}


@app.post("/track_objects")
async def set_track_objects(
    data: dict,
    current_user: schemas.User = Depends(oauth2.get_current_user)
):
    """
    Met à jour la liste track_objects dans config.yaml.
    data doit être un JSON du type:
      {
        "track_objects": ["cat", "person"]
      }
    """
    new_objs = data.get("track_objects")
    if not isinstance(new_objs, list):
        raise HTTPException(status_code=400, detail="track_objects must be a list.")

    # Mettre à jour config.yaml
    config = read_config()
    config["track_objects"] = new_objs
    write_config(config)

    # Émettre la mise à jour via WebSocket
    await manager.broadcast_update('track_objects', {'track_objects': new_objs})

    return {"track_objects": new_objs}


# *** Nouvel Endpoint : Obtenir la liste des objets disponibles pour le suivi ***
@app.get("/track_objects_list")
def get_track_objects_list(current_user: schemas.User = Depends(oauth2.get_current_user)):
    """
    Récupère la liste track_objects_list dans config.yaml
    et la renvoie sous forme JSON.
    """
    config = read_config()
    track_objects_list = config.get("track_objects_list", [])
    # On s’assure que c’est une liste, sinon on renvoie au moins []
    if not isinstance(track_objects_list, list):
        track_objects_list = []
    return {"track_objects_list": track_objects_list}

# Endpoint pour mettre à jour la liste des objets disponibles (optionnel)
@app.post("/track_objects_list")
async def set_track_objects_list(
    data: dict,
    current_user: schemas.User = Depends(oauth2.get_current_user)
):
    """
    Met à jour la liste track_objects_list dans config.yaml.
    data doit être un JSON du type:
      {
        "track_objects_list": ["cat", "person", "dog"]
      }
    """
    new_list = data.get("track_objects_list")
    if not isinstance(new_list, list):
        raise HTTPException(status_code=400, detail="track_objects_list must be a list.")

    # Mettre à jour config.yaml
    config = read_config()
    config["track_objects_list"] = new_list
    write_config(config)

    # Émettre la mise à jour via WebSocket
    await manager.broadcast_update('track_objects_list', {'track_objects_list': new_list})

    return {"track_objects_list": new_list}


@app.get("/records_list")
def records_list(current_user: schemas.User = Depends(oauth2.get_current_user)):
    """
    Renvoie la liste des vidéos dans /Videos/records, triées par date de création desc,
    en format JSON : [{"filename": "...", "ctime": 12345678, "is_today": True/False}, ...]
    """
    folder = "/home/raspi/Videos/records"
    if not os.path.exists(folder):
        return []

    files = [f for f in os.listdir(folder) if f.endswith(".mp4")]
    out = []
    for fname in files:
        path = os.path.join(folder, fname)
        st   = os.stat(path)
        out.append({
            "filename": fname,
            "ctime": st.st_ctime
        })
    # Tri desc sur ctime
    out.sort(key=lambda x: x["ctime"], reverse=True)
    return out


@app.get("/records/video/{filename}", response_class=FileResponse)
def get_record_video(
    filename: str,
    current_user: schemas.User = Depends(oauth2.get_current_user)
):
    """
    Retourne la vidéo en question (fichier MP4) si elle existe,
    et si l'utilisateur est bien connecté.
    """
    filename = unquote(filename)  # Décoder les caractères spéciaux

    # Rechercher dans records et archives
    records_folder = "/home/raspi/Videos/records"
    archive_folder = "/home/raspi/Videos/archives"
    
    path = os.path.join(records_folder, filename)
    if not os.path.exists(path):
        path = os.path.join(archive_folder, filename)
        if not os.path.exists(path):
            raise HTTPException(status_code=404, detail="Video not found")

    return FileResponse(path, media_type="video/mp4")


@app.get("/archive")
def archive_endpoint(
    filename: str,
    current_user: schemas.User = Depends(oauth2.get_current_user)
):
    """
    Déplace la vidéo 'filename' de /home/raspi/Videos/records vers /home/raspi/Videos/archives.
    """
    # Décoder les caractères spéciaux
    filename = unquote(filename.replace("+", "%2B"))
    
    success = archive_video(filename)
    if not success:
        raise HTTPException(status_code=404, detail=f"Video not found or error archiving: {filename}")

    return RedirectResponse(url="/", status_code=302)


@app.get("/delete")
def delete_video(
    filename: str,
    current_user: schemas.User = Depends(oauth2.get_current_user)
):
    """
    Supprime une vidéo dans le dossier records ou archives.
    """
    # Décoder les caractères spéciaux
    filename = unquote(filename.replace("+", "%2B"))

    records_folder = "/home/raspi/Videos/records"
    archive_folder = "/home/raspi/Videos/archives"

    # Chercher d'abord dans records
    path = os.path.join(records_folder, filename)
    if not os.path.exists(path):
        # Sinon chercher dans archives
        path = os.path.join(archive_folder, filename)
        if not os.path.exists(path):
            print(f"Fichier non trouvé : {filename}")
            raise HTTPException(status_code=404, detail=f"Video not found: {filename}")

    try:
        os.remove(path)
        print(f"Vidéo supprimée : {filename}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Erreur lors de la suppression : {e}")

    return RedirectResponse(url="/", status_code=302)
