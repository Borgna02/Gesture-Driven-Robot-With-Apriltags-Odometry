FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    xz-utils \
    tar \
    git \
    x11vnc \
    xvfb \
    fluxbox \
    libgl1 \
    libxrender1 \
    libxi6 \
    libxtst6 \
    libglib2.0-0 \
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-xfixes0 \
    libxkbcommon-x11-0 \
    libwebpmux3 \
    libqt5x11extras5 \
    python3 \
    python3-pip \
    python3-zmq \
    python3-cbor2 \
    libsodium23 \
    && rm -rf /var/lib/apt/lists/*

# Copia il file di archivio di CoppeliaSim (senza riferimenti a versioni)
COPY CoppeliaSim.tar.xz /tmp/

RUN mkdir -p /opt/coppeliasim && \
    tar -xJf /tmp/CoppeliaSim.tar.xz -C /opt/coppeliasim --strip-components=1 && \
    rm /tmp/CoppeliaSim.tar.xz

# Copia la scena PioneerScene.ttt nella directory di CoppeliaSim
COPY PioneerScene.ttt /opt/coppeliasim/

# Clona noVNC e websockify per l'interfaccia grafica via browser
RUN git clone https://github.com/novnc/noVNC.git /opt/novnc && \
    git clone https://github.com/novnc/websockify.git /opt/websockify

# Espone le porte:
#   5900: VNC
#   6080: noVNC (web)
#   23000: API remote di CoppeliaSim (ZeroMQ)
EXPOSE 5900 6080 23000

COPY start.sh /start.sh
RUN chmod +x /start.sh

CMD ["/start.sh"]
