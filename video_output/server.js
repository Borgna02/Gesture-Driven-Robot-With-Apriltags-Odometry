const express = require('express');
const bodyParser = require('body-parser');
const WebSocket = require('ws');

const app = express();
const port = process.env.PORT || 3000;
const wsPort = 3001;

// Configurazione WebSocket Server
const wss = new WebSocket.Server({ 
    port: wsPort,
    host: '0.0.0.0' 
});
// Middleware per le immagini
app.use(bodyParser.raw({
    type: 'image/jpeg',
    limit: '10mb'
}));

let latestImage = null;
let processedImage = null;

// WebSocket broadcast
function broadcast() {
    wss.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send('new-frame');
        }
    });
}

// Endpoint POST per le immagini
app.post('/video', (req, res) => {
    latestImage = req.body;
    broadcast();
    res.status(200).send('Image received');
});

// Endpoint GET per l'immagine
app.get('/video', (req, res) => {
    if (!latestImage) {
        return res.status(404).send('No image available');
    }
    res.type('image/jpeg').send(latestImage);
});

app.post('/processed_video', (req, res) => {
    processedImage = req.body;
    res.status(200).send('Processed image received');
});


// Aggiungi questo endpoint GET per il video processato
app.get('/processed_video', (req, res) => {
    if (!processedImage || processedImage.length === 0) {
        const emptyImage = Buffer.from(
            'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNkYAAAAAYAAjCB0C8AAAAASUVORK5CYII=',
            'base64'
        );
        return res.type('image/png').send(emptyImage);
    }
    res.type('image/jpeg').send(processedImage);
});


// Pagina HTML con client-side code
app.get('/', (req, res) => {
    res.send(`
    <!DOCTYPE html>
    <html>
    <head>
        <title>Dual Video Stream</title>
        <style>
            body { 
                margin: 0; 
                padding: 0;
                background: #000;
                display: flex;
                flex-direction: column;
                height: 100vh;
            }
            .video-container {
                width: 100vw;
                height: 50vh;
                position: relative;
                border-bottom: 2px solid #333;
            }
            .video-label {
                position: absolute;
                top: 10px;
                left: 10px;
                color: white;
                background: rgba(0,0,0,0.5);
                padding: 5px 10px;
                border-radius: 5px;
                z-index: 1;
            }
            #videoStream {
                border-bottom: 2px solid #333;
            }
            img {
                width: 100%;
                height: 100%;
                object-fit: contain;
                display: block;
            }
        </style>
    </head>
    <body>
        <div class="video-container">
            <div class="video-label">Original Video</div>
            <img id="videoStream" src="/video" />
        </div>
        <div class="video-container">
            <div class="video-label">Processed Video</div>
            <img id="processedVideoStream" src="/processed_video" />
        </div>

        <script>
            const streams = {
                original: {
                    element: document.getElementById('videoStream'),
                    lastUpdate: 0
                },
                processed: {
                    element: document.getElementById('processedVideoStream'),
                    lastUpdate: 0
                }
            };

            const host = window.location.hostname;
            const ws = new WebSocket('ws://' + host + ':${wsPort}');

            function updateVideoStream(type) {
                const endpoint = type === 'processed' ? '/processed_video' : '/video';
                const stream = streams[type];

                fetch(endpoint + '?ts=' + Date.now())
                    .then(response => {
                        if (!response.ok) return;
                        return response.blob();
                    })
                    .then(blob => {
                        if (!blob) return;
                        
                        const url = URL.createObjectURL(blob);
                        stream.element.onload = () => {
                            URL.revokeObjectURL(url);
                            stream.lastUpdate = Date.now();
                        };
                        stream.element.src = url;
                    })
                    .catch(error => {
                        console.error(type + ' stream error:', error);
                    });
            }

            ws.onmessage = () => {
                updateVideoStream('original');
                updateVideoStream('processed');
            };

            // Backup polling
            setInterval(() => {
                const now = Date.now();
                Object.entries(streams).forEach(([type, data]) => {
                    if (now - data.lastUpdate > 200) {
                        updateVideoStream(type);
                    }
                });
            }, 100);

            // Initial load
            updateVideoStream('original');
            updateVideoStream('processed');
        </script>
    </body>
    </html>
    `);
});

// Avvio server HTTP
app.listen(port, '0.0.0.0', () => {
    console.log(`HTTP server running on port ${port}`);
});

// Avvio server WebSocket
wss.on('listening', () => {
    console.log(`WebSocket server running on port ${wsPort}`);
});