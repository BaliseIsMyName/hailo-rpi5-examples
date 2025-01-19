import * as mediasoupClient from 'mediasoup-client';
import io from 'socket.io-client';

const socket = io('https://www.boissard.net', { path: '/mediasoup/socket.io' });

socket.on('connect', () => {
  console.log('[CLIENT] Socket connected, id =', socket.id);
});

socket.emit('consume', async (response) => {
  console.log('[CLIENT] Réponse du serveur:', response);
  if (response.error) {
    console.error('[CLIENT] consume error:', response.error);
    return;
  }
  
  const { routerRtpCapabilities, transportParams, consumerParams } = response;
  if (!routerRtpCapabilities || !transportParams || !consumerParams) {
    console.error('[CLIENT] Paramètres manquants dans la réponse du serveur');
    return;
  }
  
  try {
    const device = new mediasoupClient.Device();
    await device.load({ routerRtpCapabilities });
    console.log('[CLIENT] Device loaded successfully');
    
    const recvTransport = device.createRecvTransport(transportParams);
    console.log('[CLIENT] Receiving transport created:', recvTransport.id);
    
    recvTransport.on('connect', ({ dtlsParameters }, callback, errback) => {
      console.log('[CLIENT] Transport connect event, dtlsParameters:', dtlsParameters);
      socket.emit('transport-connect', { dtlsParameters }, (ack) => {
        console.log('[CLIENT] Ack from transport-connect:', ack);
        if (ack && ack.status === 'ok') {
          callback();
        } else {
          errback(new Error('Transport connect failed'));
        }
      });
    });
    
    const consumer = await recvTransport.consume({
      id: consumerParams.id,
      producerId: consumerParams.producerId,
      kind: consumerParams.kind,
      rtpParameters: consumerParams.rtpParameters,
      appData: {}
    });
    console.log('[CLIENT] Consumer created, id:', consumer.id);
    console.log('[CLIENT] Consumer RTP parameters:', consumer.rtpParameters);
    
    // Diagnostiquer la track
    console.log('[CLIENT] Consumer track:', consumer.track);
    console.log('[CLIENT] Track state:', consumer.track.readyState, 'Enabled:', consumer.track.enabled);
    
    consumer.track.onended = () => {
      console.log('[CLIENT] The video track has ended');
    };
    
    // Tenter d'obtenir des stats sur le consumer
    try {
      const stats = await consumer.getStats();
      console.log('[CLIENT] Consumer stats:', stats);
    } catch (e) {
      console.error('[CLIENT] getStats error:', e);
    }
    
    const stream = new MediaStream([consumer.track]);
    console.log('[CLIENT] Created MediaStream from consumer track');

    const videoElem = document.getElementById('myVideo');
    if (videoElem) {
      videoElem.srcObject = stream;
      console.log('[CLIENT] MediaStream attached to <video> element');
      videoElem.play().then(() => {
        console.log('[CLIENT] Video playback started');
      }).catch((err) => {
        console.error('[CLIENT] Video playback error:', err);
      });
    } else {
      console.error('[CLIENT] <video id="myVideo"> element not found');
    }
    
    socket.emit('consumer-resume', {}, () => {
      console.log('[CLIENT] Consumer resume sent');
    });
    
    recvTransport.on('connectionstatechange', (state) => {
      console.log('[CLIENT] Receiving transport state change:', state);
    });
    
  } catch (error) {
    console.error('[CLIENT] Error during mediasoup-client setup:', error);
  }
});
