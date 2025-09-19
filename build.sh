docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --pull \
  --no-cache \
  -f .devcontainer/Dockerfile2 \
  -t aatmaj9/sookshma:5.0 \
  --push \
  .

