FROM ros:humble-ros-core@sha256:854e48183c14881767553657ca73602fe7bb5d7f265f744c5ea66ef88bb59955
RUN mkdir /root/voraus-ros-bridge
COPY package.xml /root/voraus-ros-bridge/
COPY ./install/voraus-ros-bridge/ /root/voraus-ros-bridge/install/voraus-ros-bridge/
COPY ./launch/* /root/voraus-ros-bridge/install/voraus-ros-bridge/share/voraus-ros-bridge/
COPY ./voraus_interfaces/install/voraus_interfaces/lib/* /opt/ros/humble/lib/
RUN chmod +x /root/voraus-ros-bridge/install/voraus-ros-bridge/lib/voraus-ros-bridge/voraus-ros-bridge

