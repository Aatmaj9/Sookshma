#!/bin/bash
set -e

COMPOSE_FILE=".devcontainer/docker-compose.yml"

docker compose -f .devcontainer/docker-compose.yml up -d sookshma
docker compose -f .devcontainer/docker-compose.yml exec sookshma bash
