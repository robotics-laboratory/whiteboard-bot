# Getting started

## Docker
We use docker for two purposes:
- Unified isolated environment for easy development and tests
- Dependency and code delivery system

All prebuild base images are stored at our registry ```cr.yandex/crp8hpfj5tuhlaodm4dl```.
At this moment only ubuntu 20.04 is supported.

### Pull and run
```
# dev/amd64
docker-compose up -d wbb-amd64

# raspberry/arm64v8
docker-compose up -d wbb-arm64v8

# just run another shell (preferred)
# exit doesn't stop the container
docker exec -it wbb bash

# stop container
docker stop truck
```
### Build and push
If you need to build container, follow this steps.
Up version in ```docker-compose.yaml``` if orresponding dockerfile is changed.

```
docker-compose build
docker-compose push
```