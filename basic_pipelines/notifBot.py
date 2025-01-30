import requests

def send_telegram_message(message):
    bot_token = ""
    chat_id = ""
    url = f"https://api.telegram.org/bot{bot_token}/sendMessage"
    payload = {
        "chat_id": chat_id,
        "text": message
    }
    response = requests.post(url, json=payload)
    if response.status_code == 200:
        print("Message envoyé avec succès !")
    else:
        print("Erreur :", response.text)


# def get_chat_id(bot_token):
#     url = f"https://api.telegram.org/bot{bot_token}/getUpdates"
#     response = requests.get(url)

#     if response.status_code == 200:
#         data = response.json()
#         if "result" in data and len(data["result"]) > 0:
#             chat_id = data["result"][-1]["message"]["chat"]["id"]
#             print(f"Votre Chat ID est : {chat_id}")
#         else:
#             print("Aucune mise à jour trouvée. Envoyez un message à votre bot depuis Telegram pour récupérer le Chat ID.")
#     else:
#         print(f"Erreur : {response.status_code}, {response.text}")

# # Remplacez "VOTRE_BOT_TOKEN" par le token fourni par BotFather
# bot_token = ""
# get_chat_id(bot_token)
