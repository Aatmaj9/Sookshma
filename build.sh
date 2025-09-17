docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --pull \
  --no-cache \
  -f .devcontainer/Dockerfile1 \
  -t aatmaj9/sookshma:4.0 \
  --push \
  .

