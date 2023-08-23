sudo docker buildx create --name nimbus-builder 
sudo docker buildx use nimbus-builder
sudo docker run --privileged --rm tonistiigi/binfmt --install all
sudo docker buildx inspect --bootstrap


sudo docker buildx build --platform linux/amd64,linux/arm64 -t cognimbus/ros2-legs-detector:latest --push .






