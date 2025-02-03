
FROM ubuntu:jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y

RUN apt-get install python3.11 -y

RUN apt-get install pip -y

RUN apt-get install git -y

RUN pip install git+https://gitlab.com/Tetsuo-tek/PySketch@211f554a9d2812995c2abb5e00b20cd712e19696
RUN pip install git+https://gitlab.com/Tetsuo-tek/pck@036e9c8a164580fba33938c5bf04cb259913580d

RUN apt-get install -y \
        git \
        build-essential \
        libogg-dev \
        libssl-dev \
        libvorbis-dev \
        libfftw3-dev \
        libespeak-dev \
        portaudio19-dev \
	libflac-dev \
        libflac++-dev \
        libvlc-dev \
        libzbar-dev \
        libopencv-dev \
        libi2c-dev \
        libstatgrab-dev \
        screen \
        redis 


RUN apt-get clean
RUN cd
RUN pwd
RUN pck --install-base /root


ENV  PCK_ROOT=/root
ENV  PCK_HOME=$PCK_ROOT/pck
ENV  PCK_BIN=$PCK_HOME/bin
ENV  PATH=$PATH:$PCK_BIN

RUN pck --check-base



RUN mkdir /opt/Sk && \
    mkdir /opt/Sk/src && \
    mkdir /opt/Sk/bin && \
    mkdir /opt/Sk/data

WORKDIR /opt/Sk/src

RUN git clone https://gitlab.com/Tetsuo-tek/SpecialK.git && \
    cd SpecialK && \
    git checkout c00bc2a0877dfe4f5fb8faedc897e1c019202ed0 && \
    cd .. && \
    git clone https://gitlab.com/Tetsuo-tek/SkMake.git && \
    cd SkMake && \
    git checkout bb1e6fd5980fef5771e4bc54c268fcfa8812a32a && \
    cd .. && \
    git clone https://gitlab.com/Tetsuo-tek/SkRobot.git && \
    cd SkRobot && \
    git checkout a2ea9c7590048eaa4fd8ce851c194ffab4789cef


WORKDIR /opt/Sk/src/SkMake
RUN rm -f Makefile && \
    mv Makefile-linux.dist Makefile && \
    make && \
    mv ./SkMake.bin /opt/Sk/bin/SkMake.bin && \
    rm -rf ./build
    

WORKDIR /opt/Sk/src/SkRobot
RUN /opt/Sk/bin/SkMake.bin -m skmake-ubuntu-nocuda.json -c 8
RUN    mv ./SkRobot.bin /opt/Sk/bin/SkRobot.bin
RUN    rm -rf ./build
    
RUN ln -s /opt/Sk/bin/SkMake.bin /usr/local/bin/skmake && \
    ln -s /opt/Sk/bin/SkRobot.bin /usr/local/bin/robot 
   
ENV SK_HOME=/opt/Sk
ENV SK_SRC=$SK_HOME/src
ENV SK_BIN=$SK_HOME/bin
ENV SK_ETC=$SK_HOME/etc
ENV SK_SHR=$SK_HOME/share
ENV SK_VAR=$SK_HOME/var
ENV PATH=$PATH:$SK_BIN

RUN mkdir -p /root/home/launch

WORKDIR /root/home/launch

EXPOSE 9000
EXPOSE 9001

CMD ["SkRobot.bin"]

