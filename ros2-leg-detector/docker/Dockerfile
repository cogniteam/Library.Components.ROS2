FROM ros:foxy

RUN apt-get update && apt install python3-pip -y && \
    apt-get install qtbase5-dev -y && \
    apt-get install qtdeclarative5-dev -y && \
    pip3 install pykalman  && pip3 install scipy && \
    apt install python3-pykdl -y

RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && git checkout 3.4.12 && cd ~ 


RUN git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && git checkout 3.4.12 && cd ~

WORKDIR /opencv
RUN mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D INSTALL_C_EXAMPLES=ON \
        -D INSTALL_PYTHON_EXAMPLES=ON \
        -D WITH_TBB=ON \
        -D WITH_V4L=ON \
        -D WITH_QT=ON \
        -D WITH_OPENGL=ON \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        -D BUILD_EXAMPLES=ON .. 

WORKDIR /opencv/build/

RUN make -j12 && make install && \
        sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf' && \
        sudo ldconfig && cd ~ 


WORKDIR /leg_detector_ws/
RUN mkdir src && cd src && git clone https://github.com/mowito/ros2_leg_detector.git && cd ..
COPY ./cogniteam_leg_detector.py /leg_detector_ws/src/ros2_leg_detector/src/leg_detector/launch/

WORKDIR /leg_detector_ws/
RUN . /opt/ros/foxy/setup.sh && colcon build

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

