// server.js
const express = require('express');
const http = require('http'); // ou https si vous le souhaitez
const socketIO = require('socket.io');
const mediasoup = require('mediasoup');

let worker, router, producerTransport, videoProducer;

async function runMediasoup() {
  try {
    // 1) Créer un worker
    worker = await mediasoup.createWorker({
      rtcMinPort: 40000,
      rtcMaxPort: 49999
    });
    worker.on('died', () => {
      console.error('mediasoup worker died, exiting...');
      process.exit(1);
    });

    // 2) Créer un router avec la configuration du codec vidéo
    router = await worker.createRouter({
      mediaCodecs: [
        {
          kind: 'video',
          mimeType: 'video/H264',
          clockRate: 90000,
          parameters: { 'packetization-mode': 1 }
        }
      ]
    });
    console.log('[SERVER] Router RTP Capabilities:', router.rtpCapabilities);

    // 3) Créer un PlainTransport pour la réception du flux RTP (GStreamer)
    producerTransport = await router.createPlainTransport({
      listenIp: '0.0.0.0',
      rtcpMux: false,
      comedia: true
    });
    console.log('[SERVER] ProducerTransport:');
    console.log('  RTP port =', producerTransport.tuple.localPort);
    console.log('  RTCP port =', producerTransport.rtcpTuple.localPort);

    // 4) Créer le Producer (videoProducer)
    // Ce producer attendra que GStreamer commence à envoyer le flux RTP.
    videoProducer = await producerTransport.produce({
      kind: 'video',
      rtpParameters: {
        encodings: [{ ssrc: 123456 }],
        codecs: [{
          mimeType: 'video/H264',
          clockRate: 90000,
          payloadType: 96,
          parameters: { 'packetization-mode': 1 }
        }]
      }
    });
    console.log('[SERVER] videoProducer.id =', videoProducer.id);

    // (Optionnel) Activer des traces pour surveiller l'activité du producer
    videoProducer.on('trace', (traceData) => {
      console.log('[SERVER] Producer trace:', traceData);
    });

    // (Optionnel) Récupérer périodiquement les statistiques du Producer
    setInterval(async () => {
      try {
        const stats = await videoProducer.getStats();
        console.log('[SERVER] Producer stats:', stats);
      } catch (error) {
        console.error('[SERVER] Error fetching producer stats:', error);
      }
    }, 5000);

  } catch (err) {
    console.error('[SERVER] Erreur dans runMediasoup:', err);
  }
}

runMediasoup(); // Démarrer mediasoup

// 5) Mettre en place Express et Socket.IO
const app = express();
const server = http.createServer(app);
const io = socketIO(server, { path: '/mediasoup/socket.io' });

// Pour tester, route de test simple
app.get('/mediasoup/test', (req, res) => {
  res.send('Hello from mediasoup server');
});

// Gestion des connexions Socket.IO
io.on('connection', async (socket) => {
  console.log('[SERVER] Nouvelle connexion client, socket.id =', socket.id);

  // Le client demande à consommer le flux
  socket.on('consume', async (callback) => {
    try {
      // Créer un WebRtcTransport pour le consommateur
      const webRtcTransport = await router.createWebRtcTransport({
        listenIps: [{ ip: '0.0.0.0', announcedIp: null }], // En production, précisez announcedIp
        enableTcp: true,
        enableUdp: true
      });
      console.log('[SERVER] WebRtcTransport créé pour le client, id =', webRtcTransport.id);
      console.log('[SERVER] Transport ICE parameters:', webRtcTransport.iceParameters);
      console.log('[SERVER] Transport DTLS parameters:', webRtcTransport.dtlsParameters);

      // Créer le consumer reliant le flux vidéo
      const consumer = await webRtcTransport.consume({
        producerId: videoProducer.id,
        rtpCapabilities: router.rtpCapabilities,
        paused: false
      });
      console.log('[SERVER] Consumer créé, id =', consumer.id);
      console.log('[SERVER] Consumer RTP parameters:', consumer.rtpParameters);

      // Envoyer au client les paramètres du transport et du consumer
      callback({
        routerRtpCapabilities: router.rtpCapabilities,
        transportParams: {
          id: webRtcTransport.id,
          iceParameters: webRtcTransport.iceParameters,
          iceCandidates: webRtcTransport.iceCandidates,
          dtlsParameters: webRtcTransport.dtlsParameters
        },
        consumerParams: {
          id: consumer.id,
          producerId: consumer.producerId,
          kind: consumer.kind,
          rtpParameters: consumer.rtpParameters,
        }
      });

      // Gérer l’événement de connexion du transport
      socket.on('transport-connect', async (data, ack) => {
        console.log('[SERVER] transport-connect reçu, dtlsParameters :', data.dtlsParameters);
        await webRtcTransport.connect({ dtlsParameters: data.dtlsParameters });
        console.log('[SERVER] Transport connecté pour le client');
        if (typeof ack === 'function') ack({ status: 'ok' });
      });

      socket.on('consumer-resume', async () => {
        await consumer.resume();
        console.log('[SERVER] Consumer resumé');
      });

    } catch (err) {
      console.error('[SERVER] Erreur dans consume:', err);
      callback({ error: err.message });
    }
  });
});

server.listen(3000, () => {
  console.log('[SERVER] mediasoup server listening on port 3000');
});
