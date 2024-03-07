# TrajOpt Docker

## Create the Docker image
### Download
Download a pre-built Docker image for this bridge from the container registry:

```
docker login ghcr.io
docker pull ghcr.io/tesseract-robotics/trajopt_deploy:<tag>
```

### Build
Build the Docker image using `docker-compose`:

```commandLine
cd docker
docker compose build
```

## Run the docker image
Run the Docker image using `docker-compose`:

```commandLine
cd docker
CURRENT_UID=$(id -u):$(id -g) docker compose up
```

> Note: by default the docker image runs as a non-root user, so the environment variable `CURRENT_UID` must be supplied as shown above
