FROM node:18-alpine

WORKDIR /app

RUN echo '{"name": "video-output", "dependencies": {' > package.json && \
    echo '"express": "^4.18.2", "body-parser": "^1.20.2", "ws": "^8.16.0"' >> package.json && \
    echo '}, "scripts": {"start": "node server.js"}}' >> package.json

RUN npm install && npm cache clean --force

COPY server.js .

EXPOSE 3000 3001
CMD ["npm", "start"]