# /home/raspi/hailo-rpi5-examples/basic_pipelines/app/main.py

from fastapi import FastAPI, Depends, HTTPException, status, Request, Response
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

from . import models, schemas, crud, utils, database, oauth2
from .config import ADMIN_PASSWORD, STREAM_URL  # Importer STREAM_URL

import yaml
import shutil
import os

# Emplacement du fichier config.yaml
CONFIG_PATH = "/home/raspi/hailo-rpi5-examples/basic_pipelines/config.yaml"

def read_config():
    """Lit le contenu de config.yaml et retourne un dictionnaire Python."""
    with open(CONFIG_PATH, "r") as f:
        return yaml.safe_load(f)

def write_config(config_data):
    """Écrit le dictionnaire Python dans config.yaml."""
    with open(CONFIG_PATH, "w") as f:
        yaml.safe_dump(config_data, f)

def archive_video(filename: str, from_folder="/home/raspi/Videos/records", archive_folder="/home/raspi/Videos/archives"):
    """
    Déplace le fichier 'filename' de from_folder vers archive_folder.
    Retourne True si ok, False sinon.
    """
    src = os.path.join(from_folder, filename)
    dst = os.path.join(archive_folder, filename)
    
    if not os.path.exists(src):
        return False
    
    # Créer le dossier d'archives s'il n'existe pas
    os.makedirs(archive_folder, exist_ok=True)

    try:
        shutil.move(src, dst)
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

# Gestionnaire d'exceptions pour rediriger les 401 vers /login
@app.exception_handler(StarletteHTTPException)
async def http_exception_handler(request: Request, exc: StarletteHTTPException):
      if exc.status_code == status.HTTP_401_UNAUTHORIZED:
            return RedirectResponse(url="/loginN", status_code=status.HTTP_302_FOUND)
      return RedirectResponse(url="/error", status_code=exc.status_code)  # Optionnel: gérer d'autres erreurs

# Page d'accueil (login requis)
@app.get("/", response_class=HTMLResponse)
async def read_home(request: Request, current_user: schemas.User = Depends(oauth2.get_current_user)):
    folder = "/home/raspi/Videos/records"
    videos = []
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

    return templates.TemplateResponse("index.html", {
        "request": request,
        "user": current_user,
        "videos": videos
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

@app.post("/camera_mode")
def set_camera_mode(
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

    config = read_config()
    # On s'assure que camera_movement existe bien
    if "camera_movement" not in config:
        config["camera_movement"] = {}
    config["camera_movement"]["mode"] = new_mode

    write_config(config)
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
def set_track_objects(
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

    # On met à jour config.yaml
    config = read_config()
    config["track_objects"] = new_objs
    write_config(config)

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
def set_track_objects_list(
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

    # Optionnel : Ajouter des validations supplémentaires ici (ex: vérifier les types)

    # Mettre à jour config.yaml
    config = read_config()
    config["track_objects_list"] = new_list
    write_config(config)

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
    folder = "/home/raspi/Videos/records"
    path = os.path.join(folder, filename)
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
    Puis redirige vers la page d'accueil.
    """
    success = archive_video(filename)
    if not success:
        raise HTTPException(status_code=404, detail="Video not found or error archiving")

    # On retourne à la page d'accueil pour recharger la liste
    return RedirectResponse(url="/", status_code=302)
