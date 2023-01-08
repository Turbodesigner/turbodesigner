ARG VARIANT="3.10-bullseye"
FROM mcr.microsoft.com/vscode/devcontainers/python:0-${VARIANT}

# [Choice] Node.js version: none, lts/*, 16, 14, 12, 10
ARG NODE_VERSION="none"
RUN if [ "${NODE_VERSION}" != "none" ]; then su vscode -c "umask 0002 && . /usr/local/share/nvm/nvm.sh && nvm install ${NODE_VERSION} 2>&1"; fi

RUN apt-get update -y && \
  apt install -y libgl1-mesa-glx && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN pip install --pre cadquery

# [Optional] If your pip requirements rarely change, uncomment this section to add them to the image.
COPY requirements.txt /tmp/pip-tmp/
RUN pip install -r /tmp/pip-tmp/requirements.txt && rm -rf /tmp/pip-tmp
RUN pip install jupyter-cadquery==3.4.0 cadquery-massembly==1.0.0rc0

# Install SU2
RUN mkdir -p /usr/local/su2
RUN wget -O /usr/local/su2/su2.zip https://github.com/su2code/SU2/releases/download/v7.3.1/SU2-v7.3.1-linux64.zip
RUN unzip /usr/local/su2/su2.zip -d /usr/local/su2
RUN echo 'export PATH=/usr/local/su2/bin:$PATH' >> /home/vscode/.bashrc
RUN . /home/vscode/.bashrc